#!/bin/bash
# SPDX-FileCopyrightText: 2025 Ryoichi Sakamaki
# SPDX-License-Identifier: GPL-3.0-only
set -e
python3 -m compileall .
colcon build --packages-select mypkg