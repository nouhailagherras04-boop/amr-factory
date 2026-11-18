#!/usr/bin/env bash
pkill -f "gz sim" 2>/dev/null
cd "$(dirname "$0")"
echo "Simulateur en cours de chargement (1 a 2 minutes). NE FERMEZ PAS cette fenetre."
LIBGL_ALWAYS_SOFTWARE=1 gz sim -r factory_standalone.sdf
