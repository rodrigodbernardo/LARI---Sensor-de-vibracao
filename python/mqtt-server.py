from email import message
import paho.mqtt.client as mqtt
import time


broker_addr = '127.0.0.1'
broker_port = 1883

username = 'cliente'
password = 'cliente'
matrix = []

dados = "{\"AcX\":1000,\"AcY\":1000,\"AcZ\":1000,\"GyX\":1000,\"GyY\":1000,\"GyZ\":1000,\"Tmp\":1000}"

dados_dict = eval(dados)



def mqttConnect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    client.subscribe("/feeds/ESP_SCADA")
'''
    message = {}

    message.update({'cmd':int(input('Digite o comando: '))})
    message.update({'npk':int(input('Digite o numero de pacotes: '))})
    message.update({'spe':int(input("Digite o periodo amostral (em us): "))})

    #command = int(input("Digite o comando: "))
    #packets = int(input("Digite o numero de pacotes: "))
    #sample_period = int(input("Digite o periodo amostral (em us): "))
    
    #message = "{\"cmd\":{},\"npk\":{},\"spe\":{}}".format(command,packets,sample_period)
    print(message)
    client.publish("/feeds/SCADA_ESP", str(message))
'''
def mqttInput(client, userdata, msg):
    text_file = open("C:/Users/rodri/2022-05-04-B1-C0-001.csv", "a")
    msg.payload = msg.payload.decode("utf-8")  ### <--- ATENCAO PARA DECODIFICAR EM UTF-8

    if(msg.payload == 'fim'):
        print('fim')

    else:
        text_file.write(msg.payload + "\n")
        text_file.close()
        print('valor adicionado')
'''
    if(msg.payload == "end of transmission"):
        print('fim da transmissao')
        print(matrix)
        #salva no arquivo
    else:
        matrix.append(msg.payload)
        matrix[-1] = matrix[-1][2:-1]
        print('valor adicionado')
        print(matrix)
'''
client = mqtt.Client()
client.on_message = mqttInput
client.on_connect = mqttConnect
client.username_pw_set(username, password)



client.connect(broker_addr, broker_port)



client.loop_forever()

while(1):
    pump    = input('Bomba: ')
    classe  = input('Classe: ')