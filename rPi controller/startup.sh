#!/bin/bash
# MARS Hexapod Startup Script
# Starts the joy_controller daemon via systemd service

sudo systemctl start mars-joy.service
