<h1 align="center">
ROS node UDP client
</h1>

<p>
Esse pacote irá receber as posições de um dispositivo háptico via UDP, então aplicara esses valores no turtleSim para exemplificar uma comunicação via ROS.
</p>

<br>

<b>A aplicação espera uma string com os valores de posição dos 6 eixos da caneta, no seguinte formato:</b>
<br>
Descrição:  x      y      z      X ang. Y ang. Z ang.<br>
String:    {0.0000|0.0000|0.0000|0.0000|0.0000|0.0000}<br>

<b>Parar fazer esse pacote, instalar e executar o ROS2.</b>
<br>
Instalação: https://docs.ros.org/en/galactic/Installation.html<br>
Executar o ROS2: call C:\dev\ros2_foxy\local_setup.bat<br>

<b>Criando pacote:</b>
<br>
Na pasta do projeto: <br>
colcon build --merge-install --packages-select udp_client<br>
call install/setup.bat<br>

<b>Rodando o node:</b>
<br>
ros2 run udp_client client<br>

<b>Para testar com o turtleSim, apenas inicie um turtle node:</b>
<br>
ros2 run turtlesim turtlesim_node<br>
