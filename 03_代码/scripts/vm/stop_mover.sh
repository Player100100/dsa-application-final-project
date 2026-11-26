#!/bin/bash
pkill -9 -f "move_box_empty" >/dev/null 2>&1 || true
pkill -9 -f "move_box.sh" >/dev/null 2>&1 || true
echo movers_stopped
