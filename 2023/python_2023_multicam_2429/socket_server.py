# first of all import the socket library
import socket            
import time
import json
 
# next create a socket object
s = socket.socket()        
print ("Socket successfully created")
 
# reserve a port on your computer in our
# case it is 12345 but it can be anything
port = 12345               
 
# Next bind to the port
# we have not typed any ip in the ip field
# instead we have inputted an empty string
# this makes the server listen to requests
# coming from other computers on the network
s.bind(('', port))        
print ("socket binded to %s" %(port))
 
# put the socket into listening mode
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.listen(5)    
print ("socket is listening")           
 
# a forever loop until we interrupt it or
# an error occurs
while True:
 
# Establish connection with client.
  c, addr = s.accept()    
  print ('Got connection from', addr )
 
  data = {'TIME':time.time(), 'TARGETS_BLUE':1, 'TARGETS_RED':2}
  data_string = json.dumps(data)
  # send a thank you message to the client. encoding to send byte type.
  #c.send('Thank you for connecting'.encode())
  c.send(data_string.encode())
  # Close the connection with the client
  c.close()
   
  # Breaking once connection closed
  break

