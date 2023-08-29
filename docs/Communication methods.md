## High level explanation of the various communication methods.

### Primary & Secondary Interfaces

The UR controller offers servers that transmit robot state data and accept URScript commands. The primary interface relays both robot state data and supplementary messages, whereas the secondary interface only delivers robot state data. Primarily, this data facilitates the communication between the controller and GUI. Notably, both interfaces process URScript commands at a 10 Hz frequency, enabling the robot's remote operation without a dedicated robot program. For specifics on ports and updates, refer to: Remote Control Via TCP/IP.

### Real-time Interfaces

This operates similarly to the primary and secondary interfaces, transmitting robot state data and processing URScript commands. The distinguishing factor is its update rate. For more details on ports and updates, consult: Remote Control Via TCP/IP.

### Dashboard Server

Using the "Dashboard Server" interface, a UR robot can be manipulated remotely via simple commands transmitted over a TCP/IP socket. Predominantly, this server facilitates operations like loading, executing, pausing, and halting robot programs. It's also used to define user access levels and obtain feedback about the robot's status.

### Socket Communication

The UR robot is adept at interfacing with external equipment using the TCP/IP protocol. This enables data transfer between the robot and another device via socket communication. In this setup, the robot functions as the client, while the alternate device operates as the server. URScript introduces commands to manage sockets, including opening, closing, and data transmission in varied formats.

### XML-RPC

XML-RPC represents a Remote Procedure Call protocol that harnesses XML for data exchange across programs over sockets. Leveraging this, the UR controller can invoke methods or functions on a remote program or server, even with specific parameters, and receive structured feedback. This extends the capability of URScript by supporting complex calculations not innate to it and amalgamating other software integrations.

### RTDE (Real-Time Data Exchange)

Envisioned as an enhanced alternative to the real-time interface, RTDE permits the UR controller to broadcast custom state data and assimilate custom set-points and register data.