#ifndef LISTENER_H
#define LISTENER_H

#include <zmq.hpp>
#include <unistd.h>

class Listener{
	private:
		zmq::context_t context;
		zmq::socket_t socket;
		zmq::message_t msg;
	
	public:
		const std::string DO_NOTHING = "0";
		
		Listener(): context(1) {
		
			socket = zmq::socket_t(context, ZMQ_PULL);
			socket.bind("tcp://*:5555");
		
		}
		
		~Listener() {
			
			socket.close();
			context.shutdown();
			context.close();
			
		}
		
		std::string receive(bool wait=true) {
		
			if(!wait){
				socket.recv(msg, zmq::recv_flags::dontwait);
			}else{
				try{
					socket.recv(msg, zmq::recv_flags::none);
				}catch(int e){
					return DO_NOTHING;
				}
			}
			
        	std::string msg_string = std::string(static_cast<char *>(msg.data()), msg.size());
        	
        	return msg_string;
		
		}
};

#endif
