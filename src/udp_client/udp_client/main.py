###############################################################
#Exemplo node ROS que ecebeos valorea a publicar por UDP
#Vinicius Piovezan - Maio 2022
###############################################################

#Biblioteca para conexão UDP
from platform import node
import socket

#Bibliotecas para uso do ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import geometry_msgs.msg


#Função para iniciar conexão de cliente UDP
def connectUDP(ip = '127.0.0.1', port = 27015):
    try:
        #Tenta conectar com o servidor
        UDPClientSocket = socket.create_connection((ip, port))
        print("Conectado ao server UDP.")
    except:
        print("Erro ao conectar com o server UDP.")
        exit()
    return UDPClientSocket #Retorna objeto referente a conexão UDP

#Classe filha da classe node do ROS
class MinimalPublisher(Node):

    def __init__(self, UDPClientSocket):
        super().__init__('minimal_publisher')

        self.UDPClientSocket = UDPClientSocket

        self.topic = '/turtle1/cmd_vel' #tópico onde os valores dos eixos do dispositivo hapticoserão publicados

        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, self.topic , 10) #Publisher para enviar os dados recebidos do disp. haptico
        
        #A estrutura de dados utilizada é a geometry_msg:
            #linear
                #x - double
                #y - double
                #z - double     
            #angular
                #x - double
                #y - double
                #z - double     

        #Offset para ser somado aos valores dos eixos que chegam da caneta (função opcional)
        self.axis_offset = [0.00,0.00,0.00,0.00,0.00,0.00,]
        self.use_offset = False #Flag para sinalizar quando ooffset for ser utilizado

        #Loop de recebimento do valor UDP para envio via ROS
        while True:
            self.mainPublisher()


    def mainPublisher(self):
        bufferSize = 80 #Tamanho do buffer para recebimento dos valors UDP
        values = self.recive_udp(bufferSize) #Recebe valores UDP
        if values[0]:
            self.publish(values[1]) #Publica usando ROS
        
    #Função que irá receber os valores via UDP:
        #Em primeiro momento, os valores vão vir em um string, separados pelo caractere '|'
        #Exemplo:   x      y      z       X ang. Y ang. Z ang.
        #           {0.0000|0.0000|0.0000|0.0000|0.0000|0.0000}

    def recive_udp(self, bufferSize):
        value = self.UDPClientSocket.recvfrom(bufferSize) #Recebe o valor
        decodeValue = value[0].decode("utf-8") #Faz a decodificação, transformando em string
        # print(value)
        #Trecho responsável por verificar a intgridade da string esperada, caso não esteja nos conformes, esse pacote é ignorado
        keyA, keyB = decodeValue.find("{") , decodeValue.find("}")
        if not (keyA == 0 and keyB == len(decodeValue)-1):
            return False, False

        fullValueString = decodeValue[keyA+1:keyB] 
        splitedValue = fullValueString.split("|")  #Faz o split da string separando em um vetor 
        treaties_axis_values = self.treatAxisValues(splitedValue)
        return True , treaties_axis_values #Retorrna valores splitados

    #Método para publicar os valores recebidos e tratados, por ROS, no tópico específicado a cima
    def publish(self, value):
        message = geometry_msgs.msg.Twist() #Mensagem do tipo geometry_msg
        print(value)
        message.linear.y = (value[0]*100)*-1 
        message.linear.x = value[1]*100
        message.linear.z = value[2]*100

        message.angular.x = value[3]*100
        message.angular.y = value[4]*100
        message.angular.z = value[5]*100

        self.publisher_.publish(message)

    #Método para tratar os valores dos eixos recebidos por UDP
    def treatAxisValues(self, axis_values):
        
        treaties_axis_values = list(map(float, axis_values)) #Converte todos os valores do vetor em float

        if self.use_offset:
            treaties_axis_values = [ round((a + b),6) for a, b in zip(treaties_axis_values, self.axis_offset) ] #Soma com o offset

        return treaties_axis_values

def main(args=None):
    UDPClientSocket  = connectUDP() #Chama a função para iiciar a coxexão
    rclpy.init(args=args)
    
    minimal_publisher = MinimalPublisher(UDPClientSocket)

    rclpy.spin(minimal_publisher)

    # minimal_publisher.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    print("Iniciando...")
    main()