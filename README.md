# APIRESTful-per-la-realizzazione-di-una-piattaforma-RaaS
## Descrizione del progetto
Questa repository contiene il codice sviluppato per la tesi "Implementazione di API RESTful per la realizzazione di una piattaforma Robot as a Service". 
L'obiettivo del progetto è fornire un'interfaccia RESTful per il controllo remoto di un TurtleBot 4, utilizzando ROS2 e il framework Flask.

## Prerequisiti
Prima di avviare il progetto, si consiglia di utilizzare ubuntu versione 22.04 per la perfetta compatibilità con ROS2 Humble, e assicurati di aver installato:
- ROS2 Humble [Documentazione di ROS2 Humble](https://docs.ros.org/en/humble/index.html)
- Simulatore Turtlebot 4 [Guida all'installazione](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html)
- Gazebo GUI e Rviz2
- Flask [Documentazione di FLask](https://flask.palletsprojects.com/en/stable/)
- rclpy per la comunicazione con ROS 2
- Python 3.10
- Postman per simulare le richieste HTTP [Link per il download](https://www.postman.com/downloads/)

## Avvio del progetto
### 1. Avviare ROS 2 e la simulazione del Turtlebot 4
Aprire un terminale ed eseguire i seguenti comandi:
`source /opt/ros/humble/setup.bash`
`ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true world:=maze map:=/opt/ros/humble/share/turtlebot4_navigation/maps/maze.yaml`

### 2. Avviare il server Flask per le API RESTful
Aprire un nuovo terminale e spostarsi nella cartella contenente i file del progetto:
`cd <percorso_della_cartella>`
`python3 <nome_del_file.py>`

### 3. Test con Postman
Aprire postman e provare ad inviare richieste HTTP al robot per verificare il corretto funzionamento dell'API. Per fare ciò è molto utile visualizzare il comportamento del robot in seguito alla richiesta su GAZEBO GUI. E' molto importante prima di effettuare la richiesta accertarsi su quale porta è stato avviato il server e con quale metodo HTTP deve essere effettuata la richiesta. 

## Esempio con battery.py
### 1. Avvio di ROS 2, Gazebo e Rviz2 come descritto precedentemente
### 2. Avvio del server flask
Aprire un nuovo terminale e spostarsi nella cartella contenente i file del progetto:
`cd <percorso_della_cartella>`
`python3 battery.py`
### 3. Test con Postman
Il metodo HTTP di questa azione è GET, il server verrà avviato sulla porta 5000, quindi l'URI giusto per effettuare la richiesta sarà http://localhost:5000/battery_status. Per concludere inviare la richiesta con il tasto Send e attendere la risposta.

N.B. Negli esempi in cui bisogna inserire parametri per far si che la richiesta venga inviata correttamente, in Postman è necessario inserire i parametri nella sezione body, e poi nella sottosezione raw è possibile inserire i parametri in formato JSON.

