IPC_LFLAGS = -L$(shell rospack find ipc)/lib
IPC_BRIDGE_CXXFLAGS = -I$(shell rospack find ipc_bridge)/include

xdr: generate_msgs

generate_msgs:
	bash -c "`rospack find ipc_bridge`/generate_msgs.sh ."
