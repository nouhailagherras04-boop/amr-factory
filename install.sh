#!/usr/bin/env bash
# Installe Gazebo Harmonic + les outils Python et copie le projet dans ~/amr_projet
set -e
cd "$(dirname "$0")"
echo "=== Installation du projet AMR Factory ==="
if [ "$(lsb_release -cs 2>/dev/null)" != "noble" ]; then
  echo "ERREUR : il faut Ubuntu 24.04 (noble). Version detectee : $(lsb_release -cs 2>/dev/null)"
  exit 1
fi
echo "Le mot de passe Ubuntu va etre demande (rien ne s'affiche quand on tape : c'est normal)."
sudo apt update
sudo apt install -y curl lsb-release gnupg
if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
  sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  sudo apt update
fi
sudo apt install -y gz-harmonic python3-gz-transport13 python3-gz-msgs10 python3-opencv
mkdir -p ~/amr_projet
cp factory_standalone.sdf navigate.py start_sim.sh start_nav.sh ~/amr_projet/
chmod +x ~/amr_projet/*.sh
echo
echo "=== Installation terminee : projet copie dans ~/amr_projet ==="
gz sim --version | head -1
