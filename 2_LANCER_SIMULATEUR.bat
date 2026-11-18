@echo off
chcp 65001 >nul
title Simulateur - NE PAS FERMER
wsl -d Ubuntu-24.04 -e bash -lc "~/amr_projet/start_sim.sh"
pause
