@echo off
chcp 65001 >nul
title Installation AMR Factory
echo === INSTALLATION AMR FACTORY ===
wsl -d Ubuntu-24.04 -e true >nul 2>&1
if errorlevel 1 (
  echo Ubuntu 24.04 n est pas installe. Installation en cours...
  echo Quand Ubuntu demande un nom d utilisateur et un mot de passe : creez-les.
  echo Ensuite FERMEZ la fenetre Ubuntu ^(redemarrez le PC si Windows le demande^)
  echo puis double-cliquez de nouveau sur 1_INSTALLER.bat
  wsl --install -d Ubuntu-24.04
  pause
  exit /b
)
wsl -d Ubuntu-24.04 --cd "%~dp0linux" bash install.sh
echo.
echo Si "Installation terminee" apparait ci-dessus : double-cliquez sur 2_LANCER_SIMULATEUR.bat
pause
