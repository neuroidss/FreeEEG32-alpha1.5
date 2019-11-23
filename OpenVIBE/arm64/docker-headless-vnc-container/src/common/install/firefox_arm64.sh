#!/usr/bin/env bash
set -e

echo "Install Firefox"

function disableUpdate(){
    ff_def="$1/browser/defaults/profile"
    mkdir -p $ff_def
    echo <<EOF_FF
user_pref("app.update.auto", false);
user_pref("app.update.enabled", false);
user_pref("app.update.lastUpdateTime.addon-background-update-timer", 1182011519);
user_pref("app.update.lastUpdateTime.background-update-timer", 1182011519);
user_pref("app.update.lastUpdateTime.blocklist-background-update-timer", 1182010203);
user_pref("app.update.lastUpdateTime.microsummary-generator-update-timer", 1222586145);
user_pref("app.update.lastUpdateTime.search-engine-update-timer", 1182010203);
EOF_FF
    > $ff_def/user.js
}


#yum -y install firefox-45.7.0-2.el7.centos
#yum -y install firefox
#yum clean all
apt-get update
apt-get install -y firefox
apt-mark hold firefox
apt-get clean -y
