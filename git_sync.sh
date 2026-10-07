#!/usr/bin/env bash
set -e

MSG="$1"
if [ -z "$MSG" ]; then
    read -p "Enter commit message: " MSG
fi

if [ -z "$MSG" ]; then
    echo "[ERROR] Commit message cannot be empty."
    exit 1
fi

echo "1. Staging files..."
git add -A

echo "2. Committing locally..."
git commit -m "$MSG" || echo "[NOTE] No changes to commit"

echo "3. Pushing to origin main..."
git push origin main
echo "[SUCCESS] Synchronized: Committed and pushed to origin/main."
