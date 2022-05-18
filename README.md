<h1 align="center">
ROS node UPD client
</h1>

<p>
Esse pacote irá receber as posições de um dispositivo háptico via UDP, então aplicara esses valores no turtleSim para exemplificar uma comunicação via ROS.

A aplicação espera uma string com os valores de posição dos 6 eixos da caneta, no seguinte formato:
Descrição:  x      y      z      X ang. Y ang. Z ang.
String:    {0.0000|0.0000|0.0000|0.0000|0.0000|0.0000}

Parar fazer esse pacote, instalar e executar o ROS2.
Instalação: https://docs.ros.org/en/galactic/Installation.html
Executar o ROS2: call C:\dev\ros2_foxy\local_setup.bat 

Criando pacote:
Na pasta do projeto: 
colcon build --merge-install --packages-select udp_client
call install/setup.bat

Rodando o node:
ros2 run udp_client client

Para testar com o turtleSim, apenas inicie um turtle node:
ros2 run turtlesim turtlesim_node
</p>