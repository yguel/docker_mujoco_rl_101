#!/bin/bash
# External backup script for temporary RL workspace

set -e

TIMESTAMP=$(date '+%Y_%m_%d__%Hh%Mm%Ss')
SOURCE_DIR="/tmp/rl/mujoco"
TARGET_DIR="$HOME/rl/docker_snapshots/mujoco_snapshots"

echo "🔄 Creating backup of temporary workspace..."

if [ ! -d "$SOURCE_DIR" ]; then
    echo "❌ Source directory $SOURCE_DIR does not exist"
    exit 1
fi

# Create target directory
SOURCE_DIR="/tmp/rl/mujoco"
# Allow TARGET_DIR to be set as first argument, fallback to default
if [ -n "$1" ]; then
    TARGET_DIR="$1"
else
    TARGET_DIR="$HOME/rl/mujoco/snapshots"
fi

# Create backup (prefer zip, fallback to tar.gz)
cd /tmp
if command -v zip >/dev/null 2>&1; then
    BACKUP_NAME="${TIMESTAMP}_rl_mujoco.zip"
    if zip -r "$TARGET_DIR/$BACKUP_NAME" "rl/mujoco" >/dev/null 2>&1; then
        echo "✅ ZIP backup created: $TARGET_DIR/$BACKUP_NAME"
    else
        echo "❌ Failed to create ZIP backup"
        exit 1
    fi
else
    BACKUP_NAME="${TIMESTAMP}_rl_mujoco.tgz"
    if tar -czf "$TARGET_DIR/$BACKUP_NAME" "rl/mujoco" 2>/dev/null; then
        echo "✅ TGZ backup created: $TARGET_DIR/$BACKUP_NAME"
    else
        echo "❌ Failed to create TGZ backup"
        exit 1
    fi
fi

# Show backup size
if command -v du >/dev/null 2>&1; then
    SIZE=$(du -h "$TARGET_DIR/$BACKUP_NAME" | cut -f1)
    echo "   Size: $SIZE"
fi

echo "📁 Backup location: $TARGET_DIR/$BACKUP_NAME"