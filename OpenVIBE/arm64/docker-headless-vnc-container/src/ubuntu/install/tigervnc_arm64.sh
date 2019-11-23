#!/usr/bin/env bash
set -e

echo "Install TigerVNC server"

apt-get update

apt-get -y install tigervnc-standalone-server tigervnc-xorg-extension tigervnc-viewer

apt-get clean -y
