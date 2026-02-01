#!/bin/bash

# 1. Lancia il tuo launchfile ROS 2 in background
# Sostituisci con il tuo pacchetto e launchfile reale
echo "Avvio ROS 2 Launchfile..."
ros2 launch tiago_exam task3.launch.py &

# Salva il PID per poter chiudere tutto dopo (opzionale)
ROS_PID=$!

# 2. Attendi che le finestre GUI (Gazebo e RViz) si aprano
# Aumenta il tempo se il tuo PC è lento ad aprire Gazebo
echo "Attendo l'apertura delle GUI..."
sleep 8 

# 3. Lancia rqt_image_view in background
echo "Avvio RQT Image View..."
ros2 run rqt_image_view rqt_image_view &
sleep 2

# ---------------------------------------------------------
# 4. POSIZIONAMENTO FINESTRE (Esempio per schermo 1920x1080)
# La griglia è divisa in: 960 (metà larghezza) x 540 (metà altezza)
# ---------------------------------------------------------

# -> Gazebo: In alto a sinistra (0, 0)
# wmctrl cerca una finestra che contiene "Gazebo" nel titolo
wmctrl -r "Gazebo" -e 0,0,0,960,540

# -> RViz: In alto a destra (960, 0)
wmctrl -r "RViz" -e 0,960,0,960,540

# -> RQT Image View: In basso a sinistra (0, 540)
wmctrl -r "rqt_image_view" -e 0,0,540,960,540

# -> Il Terminale Corrente: In basso a destra (960, 540)
# :ACTIVE: seleziona la finestra del terminale che sta eseguendo questo script
wmctrl -r :ACTIVE: -e 0,960,540,960,540

echo "Layout completato!"

# Mantiene lo script attivo per non chiudere il terminale se necessario
wait $ROS_PID