#!/bin/bash

SESSIONNAME="sim2log"

rosrun catmux create_session package://sim2log_motek_launch/etc/catmux_session.yaml --tmux_config package://catmux/etc/tmux_default.conf --session_name $SESSIONNAME --overwrite sim=true
