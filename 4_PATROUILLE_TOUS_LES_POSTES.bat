@echo off
chcp 65001 >nul
title Patrouille AMR
wsl -d Ubuntu-24.04 -e bash -lc "~/amr_projet/start_nav.sh --patrol"
pause
