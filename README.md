# Robot Auction - TFG Sergi Romero Bonnin

## Mensajes
Todos los mensajes derivan de la clase abstracta Message (en Message.h).

| Tipo de mensaje |    Clase asociada   |
|:---------------:|:-------------------:|
| NEW_TASK        | NewTaskMessage      |
| LEADER_REQUEST  | BidMessage          |
| LEADER_OF_TASK  | LeaderOfTaskMessage |
| NEW_ROBOT       | NewRobotMessage     |
| BID_REQUEST     | SimpleMessage       |
| BID_FOR_TASK    | BidMessage          |
| ROBOT_ALIVE     | SimpleMessage       |
| REFUSE          | SimpleMessage       |

### Protocolo para añadir un tipo de mensaje nuevo
1. Escoger una clase existente o crear una nueva que derive de Message
2. Implementar un constructor con parámetros, un constructor a partir de un mensaje serializado y el método de serialización. Asignar el tipo.
3. Implementar un método <b> handler </b> en el servidor
4. Implementar la creación en el método <b> create_message_from ( char * msg) </b> de <it> MessageSystem </it>



## MessageSystem

Actualmente depende de la libreria RcSockets, pero el sistema está pensado para poder cambiar el manejo de sockets por cualquier alternativa.

## RobotManager

Implementa los algoritmos principales de subasta, el servidor de mensajes y el hilo de subasta.
