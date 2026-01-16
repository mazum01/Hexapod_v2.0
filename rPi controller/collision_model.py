"""
collision_model.py — S2: Learned Collision Model.

Neural network trained on FK/collision simulation data to predict collision
probability from joint angles. Faster than analytical check for real-time
gait planning, and can generalize to configurations not explicitly tested.

Training Pipeline:
    1. generate_training_data() — Sample random joint configs, run FK, check collision
    2. train_model() — Train MLP on labeled data
    3. export_to_onnx() — Export for fast inference
    
Inference:
    model = CollisionPredictor.load("collision_model.onnx")
    prob = model.predict(joint_angles)  # 0.0 = safe, 1.0 = collision

Architecture:
    Input: 18 joint angles (6 legs × 3 joints each: coxa, femur, tibia)
    Hidden: 2× Dense layers with ReLU
    Output: 6 collision probabilities (one per adjacent leg pair) + 1 body collision
    
    Total: 18 → 64 → 32 → 7 (~3k parameters, <1ms inference on Pi 5 CPU)

Created: 2026-01-10 (S2 implementation)
"""

from __future__ import annotations
import math
import json
import random
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional

import numpy as np

# Local imports
import kinematics
from kinematics import LegPose, NUM_LEGS
import collision
from collision import (
    ADJACENT_PAIRS,
    check_inter_leg_collision_with_distances,
    check_body_collision,
    get_min_dist_leg_leg_mm,
)

# -----------------------------------------------------------------------------
# Constants
# -----------------------------------------------------------------------------

# Joint angle ranges (degrees) for random sampling
# Conservative ranges based on physical limits
JOINT_RANGES = {
    'coxa':  (-45.0, 45.0),
    'femur': (-60.0, 60.0),
    'tibia': (-120.0, 0.0),  # Tibia typically bends inward (negative)
}

# Number of adjacent pairs
NUM_PAIRS = len(ADJACENT_PAIRS)  # 4

# Output dimension: 4 pair collisions + 1 body collision
OUTPUT_DIM = NUM_PAIRS + 1

# Default model path
DEFAULT_MODEL_PATH = Path(__file__).parent / "assets" / "collision_model.onnx"
DEFAULT_DATA_PATH = Path(__file__).parent / "assets" / "collision_training_data.json"

# -----------------------------------------------------------------------------
# Data Generation
# -----------------------------------------------------------------------------

@dataclass
class TrainingSample:
    """Single training sample."""
    joint_angles: list[float]  # 18 values (6 legs × 3 joints)
    pair_collisions: list[float]  # 4 values (0.0 or 1.0 per pair)
    body_collision: float  # 0.0 or 1.0
    min_distances: list[float]  # 4 values (mm, for soft labels if needed)


def random_joint_angles() -> list[LegPose]:
    """Generate random joint angles for all 6 legs."""
    poses = []
    for _ in range(NUM_LEGS):
        coxa = random.uniform(*JOINT_RANGES['coxa'])
        femur = random.uniform(*JOINT_RANGES['femur'])
        tibia = random.uniform(*JOINT_RANGES['tibia'])
        poses.append(LegPose(coxa, femur, tibia))
    return poses


def poses_to_flat(poses: list[LegPose]) -> list[float]:
    """Convert list of LegPose to flat 18-element list."""
    flat = []
    for p in poses:
        flat.extend([p.coxa, p.femur, p.tibia])
    return flat


def flat_to_poses(flat: list[float]) -> list[LegPose]:
    """Convert flat 18-element list to list of LegPose."""
    poses = []
    for i in range(NUM_LEGS):
        base = i * 3
        poses.append(LegPose(flat[base], flat[base + 1], flat[base + 2]))
    return poses


def evaluate_collision(poses: list[LegPose]) -> TrainingSample:
    """
    Run analytical collision check and return labeled sample.
    """
    # Compute FK chains
    chains = []
    for i in range(NUM_LEGS):
        chains.append(kinematics.fk_leg(i, poses[i].coxa, poses[i].femur, poses[i].tibia))
    
    # Check inter-leg collisions with distances
    _, distances, threshold = check_inter_leg_collision_with_distances(chains)
    
    # Convert to collision labels
    pair_collisions = []
    min_distances = []
    for pair in ADJACENT_PAIRS:
        dist = distances.get(pair, float('inf'))
        min_distances.append(dist)
        # Binary label: collision if distance < threshold
        pair_collisions.append(1.0 if dist < threshold else 0.0)
    
    # Check body collision
    body_col = 1.0 if check_body_collision(chains) else 0.0
    
    return TrainingSample(
        joint_angles=poses_to_flat(poses),
        pair_collisions=pair_collisions,
        body_collision=body_col,
        min_distances=min_distances,
    )


def generate_training_data(
    n_samples: int = 100000,
    seed: int = 42,
    save_path: Optional[Path] = None,
    progress_interval: int = 10000,
) -> list[TrainingSample]:
    """
    Generate training dataset by random sampling.
    
    Args:
        n_samples: Number of samples to generate
        seed: Random seed for reproducibility
        save_path: Optional path to save JSON dataset
        progress_interval: Print progress every N samples
        
    Returns:
        List of TrainingSample
    """
    random.seed(seed)
    np.random.seed(seed)
    
    samples = []
    collision_count = 0
    body_collision_count = 0
    
    print(f"Generating {n_samples:,} training samples...")
    start_time = time.time()
    
    for i in range(n_samples):
        poses = random_joint_angles()
        sample = evaluate_collision(poses)
        samples.append(sample)
        
        # Track collision rate
        if any(c > 0.5 for c in sample.pair_collisions):
            collision_count += 1
        if sample.body_collision > 0.5:
            body_collision_count += 1
        
        if (i + 1) % progress_interval == 0:
            elapsed = time.time() - start_time
            rate = (i + 1) / elapsed
            eta = (n_samples - i - 1) / rate
            print(f"  {i+1:,}/{n_samples:,} samples ({rate:.0f}/s, ETA {eta:.0f}s) "
                  f"— {collision_count} leg collisions, {body_collision_count} body collisions")
    
    elapsed = time.time() - start_time
    print(f"Generated {n_samples:,} samples in {elapsed:.1f}s")
    print(f"  Leg collision rate: {collision_count/n_samples*100:.2f}%")
    print(f"  Body collision rate: {body_collision_count/n_samples*100:.2f}%")
    
    if save_path:
        save_path = Path(save_path)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Convert to JSON-serializable format
        data = [asdict(s) for s in samples]
        with open(save_path, 'w') as f:
            json.dump(data, f)
        print(f"Saved to {save_path}")
    
    return samples


def load_training_data(path: Path) -> list[TrainingSample]:
    """Load training data from JSON file."""
    with open(path) as f:
        data = json.load(f)
    return [TrainingSample(**d) for d in data]


# -----------------------------------------------------------------------------
# Neural Network Model (PyTorch)
# -----------------------------------------------------------------------------

# Optional PyTorch import (only needed for training)
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.utils.data import Dataset, DataLoader
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    torch = None
    nn = None


# Only define PyTorch classes if torch is available
if TORCH_AVAILABLE:
    class CollisionDataset:
        """PyTorch Dataset for collision training data."""
        
        def __init__(self, samples: list[TrainingSample]):
            self.X = np.array([s.joint_angles for s in samples], dtype=np.float32)
            # Combine pair collisions and body collision into single output vector
            self.Y = np.array(
                [s.pair_collisions + [s.body_collision] for s in samples],
                dtype=np.float32
            )
            
            # Normalize inputs to [-1, 1] based on joint ranges
            self.input_mean = np.zeros(18, dtype=np.float32)
            self.input_scale = np.ones(18, dtype=np.float32)
            
            for i in range(6):
                base = i * 3
                for j, joint in enumerate(['coxa', 'femur', 'tibia']):
                    lo, hi = JOINT_RANGES[joint]
                    self.input_mean[base + j] = (lo + hi) / 2
                    self.input_scale[base + j] = (hi - lo) / 2
            
            self.X_norm = (self.X - self.input_mean) / self.input_scale
        
        def __len__(self):
            return len(self.X)
        
        def __getitem__(self, idx):
            return (
                torch.from_numpy(self.X_norm[idx]),
                torch.from_numpy(self.Y[idx])
            )


    class CollisionMLP(nn.Module):
        """
        Small MLP for collision prediction.
        
        Input: 18 normalized joint angles
        Output: 5 collision probabilities (4 leg pairs + 1 body)
        """
        
        def __init__(self, hidden_sizes: tuple[int, ...] = (64, 32)):
            super().__init__()
            
            layers = []
            in_dim = 18
            
            for h in hidden_sizes:
                layers.append(nn.Linear(in_dim, h))
                layers.append(nn.ReLU())
                in_dim = h
            
            layers.append(nn.Linear(in_dim, OUTPUT_DIM))
            layers.append(nn.Sigmoid())  # Output probabilities
            
            self.net = nn.Sequential(*layers)
        
        def forward(self, x):
            return self.net(x)


def train_model(
    samples: list[TrainingSample],
    epochs: int = 50,
    batch_size: int = 256,
    lr: float = 0.001,
    hidden_sizes: tuple[int, ...] = (64, 32),
    val_split: float = 0.1,
    device: str = "cpu",
) -> tuple:
    """
    Train collision prediction model.
    
    Returns:
        (trained_model, dataset, training_history)
    """
    if not TORCH_AVAILABLE:
        raise RuntimeError("PyTorch not available. Install with: pip install torch")
    
    # Create dataset
    dataset = CollisionDataset(samples)
    
    # Split train/val
    n_val = int(len(dataset) * val_split)
    n_train = len(dataset) - n_val
    
    indices = list(range(len(dataset)))
    random.shuffle(indices)
    train_indices = indices[:n_train]
    val_indices = indices[n_train:]
    
    train_loader = DataLoader(
        [dataset[i] for i in train_indices],
        batch_size=batch_size,
        shuffle=True,
    )
    val_loader = DataLoader(
        [dataset[i] for i in val_indices],
        batch_size=batch_size,
        shuffle=False,
    )
    
    # Create model
    model = CollisionMLP(hidden_sizes).to(device)
    
    # Binary cross-entropy loss (multi-label classification)
    criterion = nn.BCELoss()
    optimizer = optim.Adam(model.parameters(), lr=lr)
    
    history = {'train_loss': [], 'val_loss': [], 'val_accuracy': []}
    
    print(f"Training on {n_train} samples, validating on {n_val}")
    print(f"Model: {sum(p.numel() for p in model.parameters())} parameters")
    
    for epoch in range(epochs):
        # Training
        model.train()
        train_loss = 0.0
        for X_batch, Y_batch in train_loader:
            X_batch = X_batch.to(device)
            Y_batch = Y_batch.to(device)
            
            optimizer.zero_grad()
            pred = model(X_batch)
            loss = criterion(pred, Y_batch)
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item() * len(X_batch)
        
        train_loss /= n_train
        
        # Validation
        model.eval()
        val_loss = 0.0
        correct = 0
        total = 0
        
        with torch.no_grad():
            for X_batch, Y_batch in val_loader:
                X_batch = X_batch.to(device)
                Y_batch = Y_batch.to(device)
                
                pred = model(X_batch)
                loss = criterion(pred, Y_batch)
                val_loss += loss.item() * len(X_batch)
                
                # Accuracy: threshold at 0.5
                pred_binary = (pred > 0.5).float()
                correct += (pred_binary == Y_batch).sum().item()
                total += Y_batch.numel()
        
        val_loss /= n_val
        val_acc = correct / total
        
        history['train_loss'].append(train_loss)
        history['val_loss'].append(val_loss)
        history['val_accuracy'].append(val_acc)
        
        if (epoch + 1) % 10 == 0 or epoch == 0:
            print(f"Epoch {epoch+1:3d}: train_loss={train_loss:.4f}, "
                  f"val_loss={val_loss:.4f}, val_acc={val_acc:.4f}")
    
    return model, dataset, history


def export_to_onnx(
    model: nn.Module,
    dataset: CollisionDataset,
    path: Path,
):
    """Export trained model to ONNX format."""
    if not TORCH_AVAILABLE:
        raise RuntimeError("PyTorch not available")
    
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    
    model.eval()
    
    # Dummy input for tracing
    dummy_input = torch.randn(1, 18)
    
    # Export
    torch.onnx.export(
        model,
        dummy_input,
        str(path),
        input_names=['joint_angles'],
        output_names=['collision_probs'],
        dynamic_axes={
            'joint_angles': {0: 'batch'},
            'collision_probs': {0: 'batch'},
        },
        opset_version=11,
    )
    
    # Save normalization parameters alongside
    meta_path = path.with_suffix('.json')
    meta = {
        'input_mean': dataset.input_mean.tolist(),
        'input_scale': dataset.input_scale.tolist(),
        'output_labels': [f"pair_{a}_{b}" for a, b in ADJACENT_PAIRS] + ['body'],
        'threshold': 0.5,
    }
    with open(meta_path, 'w') as f:
        json.dump(meta, f, indent=2)
    
    print(f"Exported model to {path}")
    print(f"Exported metadata to {meta_path}")


# -----------------------------------------------------------------------------
# Inference (ONNX Runtime)
# -----------------------------------------------------------------------------

# Optional ONNX Runtime import
try:
    import onnxruntime as ort
    ONNX_AVAILABLE = True
except ImportError:
    ONNX_AVAILABLE = False
    ort = None


class CollisionPredictor:
    """
    Fast collision prediction using trained ONNX model.
    
    Usage:
        predictor = CollisionPredictor.load("collision_model.onnx")
        probs = predictor.predict(joint_angles)  # Returns 5 probabilities
        is_safe = predictor.is_safe(joint_angles)  # Returns bool
    """
    
    def __init__(self, session: "ort.InferenceSession", input_mean: np.ndarray,
                 input_scale: np.ndarray, threshold: float = 0.5):
        self.session = session
        self.input_mean = input_mean
        self.input_scale = input_scale
        self.threshold = threshold
        self.input_name = session.get_inputs()[0].name
    
    @classmethod
    def load(cls, model_path: Path | str) -> "CollisionPredictor":
        """Load model from ONNX file."""
        if not ONNX_AVAILABLE:
            raise RuntimeError("ONNX Runtime not available. Install with: pip install onnxruntime")
        
        model_path = Path(model_path)
        meta_path = model_path.with_suffix('.json')
        
        # Load ONNX model
        session = ort.InferenceSession(str(model_path))
        
        # Load normalization metadata
        with open(meta_path) as f:
            meta = json.load(f)
        
        return cls(
            session=session,
            input_mean=np.array(meta['input_mean'], dtype=np.float32),
            input_scale=np.array(meta['input_scale'], dtype=np.float32),
            threshold=meta.get('threshold', 0.5),
        )
    
    def predict(self, joint_angles: list[float] | np.ndarray) -> np.ndarray:
        """
        Predict collision probabilities.
        
        Args:
            joint_angles: 18 joint angles (flat list or array)
            
        Returns:
            Array of 5 probabilities: [pair_0_1, pair_1_2, pair_3_4, pair_4_5, body]
        """
        x = np.array(joint_angles, dtype=np.float32).reshape(1, 18)
        x_norm = (x - self.input_mean) / self.input_scale
        
        outputs = self.session.run(None, {self.input_name: x_norm})
        return outputs[0][0]
    
    def predict_batch(self, joint_angles_batch: np.ndarray) -> np.ndarray:
        """Predict collision probabilities for batch of configurations."""
        x = np.array(joint_angles_batch, dtype=np.float32)
        if x.ndim == 1:
            x = x.reshape(1, -1)
        x_norm = (x - self.input_mean) / self.input_scale
        
        outputs = self.session.run(None, {self.input_name: x_norm})
        return outputs[0]
    
    def is_safe(self, joint_angles: list[float] | np.ndarray,
                threshold: Optional[float] = None) -> bool:
        """
        Check if configuration is safe (no collisions predicted).
        
        Args:
            joint_angles: 18 joint angles
            threshold: Optional custom threshold (default uses model's threshold)
            
        Returns:
            True if all collision probabilities are below threshold
        """
        if threshold is None:
            threshold = self.threshold
        
        probs = self.predict(joint_angles)
        return bool(np.all(probs < threshold))
    
    def get_collision_details(self, joint_angles: list[float] | np.ndarray) -> dict:
        """
        Get detailed collision prediction with pair labels.
        """
        probs = self.predict(joint_angles)
        
        return {
            'pair_LF_LM': probs[0],
            'pair_LM_LR': probs[1],
            'pair_RF_RM': probs[2],
            'pair_RM_RR': probs[3],
            'body': probs[4],
            'any_collision': bool(np.any(probs > self.threshold)),
        }


# Singleton for cached predictor
_predictor_cache: Optional[CollisionPredictor] = None


def get_predictor(model_path: Optional[Path] = None) -> CollisionPredictor:
    """
    Get cached collision predictor (loads model on first call).
    """
    global _predictor_cache
    
    if _predictor_cache is None:
        path = model_path or DEFAULT_MODEL_PATH
        if not path.exists():
            raise FileNotFoundError(
                f"Collision model not found at {path}. "
                "Run training first: python collision_model.py --train"
            )
        _predictor_cache = CollisionPredictor.load(path)
    
    return _predictor_cache


def predict_collision_fast(joint_angles: list[float]) -> bool:
    """
    Fast collision check using learned model.
    
    Drop-in replacement for analytical check when model is trained.
    
    Returns:
        True if SAFE (no collision predicted)
    """
    try:
        predictor = get_predictor()
        return predictor.is_safe(joint_angles)
    except FileNotFoundError:
        # Fall back to analytical check if model not available
        poses = flat_to_poses(joint_angles)
        return collision.validate_pose_safety(poses)


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="S2: Learned Collision Model")
    parser.add_argument("--generate", type=int, metavar="N",
                        help="Generate N training samples")
    parser.add_argument("--train", action="store_true",
                        help="Train model on existing data")
    parser.add_argument("--epochs", type=int, default=50,
                        help="Training epochs (default: 50)")
    parser.add_argument("--test", action="store_true",
                        help="Test trained model accuracy")
    parser.add_argument("--benchmark", action="store_true",
                        help="Benchmark inference speed")
    parser.add_argument("--data-path", type=Path, default=DEFAULT_DATA_PATH,
                        help="Training data path")
    parser.add_argument("--model-path", type=Path, default=DEFAULT_MODEL_PATH,
                        help="Model output path")
    
    args = parser.parse_args()
    
    if args.generate:
        generate_training_data(
            n_samples=args.generate,
            save_path=args.data_path,
        )
    
    elif args.train:
        if not args.data_path.exists():
            print(f"Training data not found at {args.data_path}")
            print("Generate first with: python collision_model.py --generate 100000")
            return
        
        samples = load_training_data(args.data_path)
        model, dataset, history = train_model(samples, epochs=args.epochs)
        export_to_onnx(model, dataset, args.model_path)
    
    elif args.test:
        if not args.model_path.exists():
            print(f"Model not found at {args.model_path}")
            return
        
        predictor = CollisionPredictor.load(args.model_path)
        
        # Generate test samples
        print("Testing on 1000 random samples...")
        correct = 0
        total = 0
        
        for _ in range(1000):
            poses = random_joint_angles()
            joint_angles = poses_to_flat(poses)
            
            # Analytical result
            analytical_safe = collision.validate_pose_safety(poses)
            
            # Model result
            model_safe = predictor.is_safe(joint_angles)
            
            if analytical_safe == model_safe:
                correct += 1
            total += 1
        
        print(f"Agreement: {correct}/{total} ({correct/total*100:.1f}%)")
    
    elif args.benchmark:
        if not args.model_path.exists():
            print(f"Model not found at {args.model_path}")
            return
        
        predictor = CollisionPredictor.load(args.model_path)
        
        # Benchmark single inference
        poses = random_joint_angles()
        joint_angles = poses_to_flat(poses)
        
        n_iter = 10000
        start = time.perf_counter()
        for _ in range(n_iter):
            predictor.is_safe(joint_angles)
        elapsed = time.perf_counter() - start
        
        print(f"Single inference: {elapsed/n_iter*1e6:.1f} µs ({n_iter/elapsed:.0f}/s)")
        
        # Benchmark batch inference
        batch = np.array([poses_to_flat(random_joint_angles()) for _ in range(100)])
        
        n_batch_iter = 1000
        start = time.perf_counter()
        for _ in range(n_batch_iter):
            predictor.predict_batch(batch)
        elapsed = time.perf_counter() - start
        
        samples_per_sec = n_batch_iter * 100 / elapsed
        print(f"Batch (100): {elapsed/n_batch_iter*1e3:.2f} ms ({samples_per_sec:.0f} samples/s)")
        
        # Compare with analytical
        print("\nAnalytical comparison:")
        start = time.perf_counter()
        for _ in range(1000):
            collision.validate_pose_safety(poses)
        elapsed = time.perf_counter() - start
        print(f"Analytical: {elapsed:.3f} ms/sample ({1000/elapsed:.0f}/s)")
    
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
