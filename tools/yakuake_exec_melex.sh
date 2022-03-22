#!/bin/bash

sleep 2      


TERMINAL_ID_0=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.terminalIdsForSessionId 0)
qdbus org.kde.yakuake /yakuake/tabs setTabTitle 0 "Melex-dsr"

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_0"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_0"

SESSION_ID_1=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
TERMINAL_ID_1=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.terminalIdsForSessionId 1)
qdbus org.kde.yakuake /yakuake/tabs setTabTitle 1 "Cameras"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_1"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_1"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_1"

SESSION_ID_2=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
TERMINAL_ID_2=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.terminalIdsForSessionId 2)
qdbus org.kde.yakuake /yakuake/tabs setTabTitle 2 "Batery-GPS"

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_2"
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_2"

SESSION_ID_3=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
TERMINAL_ID_3=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.terminalIdsForSessionId 3)
qdbus org.kde.yakuake /yakuake/tabs setTabTitle 3 "Control-bridge-joystick"

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_3"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_3"

###

#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 0 "ssh -X pioneernuc@pioneernuc.local"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 1 "ssh -X pioneernuc@pioneernuc.local"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 2 "ssh -X pioneernuc@pioneernuc.local"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 3 "ssh -X pioneernuc@pioneernuc.local"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 4 "ssh -X pioneernuc@pioneernuc.local"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 5 "ssh -X pioneernuc@pioneernuc.local"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 6 "ssh -X pioneernuc@pioneernuc.local"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 7 "ssh -X pioneernuc@pioneernuc.local"
###

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 0 "cd /home/robocomp/robocomp/components/melexcar/agentes/idserver"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 0 "bin/idserver etc/config_melex"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 1 "cd /home/robocomp/robocomp/components/melexcar/agentes/melex_dsr"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 1 "src/melex_dsr.py"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 2 "cd /home/robocomp/robocomp/components/melexcar/agentes/monitor_dsr"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 2 "src/monitor_dsr.py"

###

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 3 "cd /home/robocomp/robocomp/components/melexcar/agentes/agent_three_cameras"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 3 "bin/agent_three_cameras etc/config"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 6 "cd /home/robocomp/robocomp/components/melexcar/agentes/agent_back_camera"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 6 "bin/agent_back_camera etc/config"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 5 "cd /home/robocomp/robocomp/components/melexcar/agentes/agent_left_camera"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 5 "bin/agent_left_camera etc/config"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 4 "cd /home/robocomp/robocomp/components/melexcar/agentes/agent_right_camera"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 4 "bin/agent_right_camera etc/config"

###

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 7 "cd /home/robocomp/robocomp/components/melexcar/agentes/gps_agent"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 7 "src/gps_agent.py"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 8 "cd /home/robocomp/robocomp/components/melexcar/agentes/battery_agent"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 8 "src/battery_agent.py"


###


qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 9 "cd /home/robocomp/robocomp/components/melexcar/agentes/agent_control"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 9 "bin/agent_control etc/config"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 10 "cd /home/robocomp/robocomp/components/melexcar/agentes/agent_bridge"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 10 "bin/agent_bridge etc/config"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 11 "cd /home/robocomp/robocomp/components/robocomp-robolab/components/hardware/external_control/joystickpublish"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 11 "bin/JoystickPublish etc/config_pioneer"

