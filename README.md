# Ultra-Wideband Swarm Ranging

The **Swarm Ranging Protocol** is a UWB-based ranging protocol that dedicates to provide a simple yet efficient ranging experience for dynamic and dense swarm networks of robots and devices. 

This repository contains the implementation of the paper:

- **Ultra-Wideband Swarm Ranging**. [Feng Shan](http://twinhorse.net/), Jiaxin Zeng, Zengbao Li, [Junzhou Luo](https://cse.seu.edu.cn/2019/0102/c23024a257045/page.htm), [Weiwei Wu](https://cse.seu.edu.cn/2019/0103/c23024a257230/page.htm). *INFOCOM 2021*. [PDF](http://twinhorse.net/papers/SZLLW-INFOCOM21p.pdf) & [CODE](https://github.com/SEU-NetSI/crazyflie-firmware/tree/archive/master/2022.05)
- **Ultra-Wideband Swarm Ranging Protocol for Dynamic and Dense Networks**. [Feng Shan](http://twinhorse.net/), Haodong Huo, Jiaxin Zeng, Zengbao Li, [Weiwei Wu](https://cse.seu.edu.cn/2019/0103/c23024a257230/page.htm), [Junzhou Luo](https://cse.seu.edu.cn/2019/0102/c23024a257045/page.htm). *IEEE/ACM Transactions on Networking, 2022*. [PDF](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9810917)

## Prerequisites

Currently the protocol implementation is based on the DW3000 UWB chip, we implemented the [DW3000 chip driver](https://github.com/SEU-NetSI/libdw3000) on Crazyflie and made a custom extension deck, which we named it the `adhoc deck`. 

The older implementation based on the DW1000 UWB chip (loco deck) can be found [here](https://github.com/SEU-NetSI/crazyflie-firmware/tree/archive/master/2022.05). Considering the older implementation is no longer actively maintained, we highly recommend you to upgrade `loco deck` to `adhoc deck`. The upgradation is very simple and straightforward, since the dw3000 and dw1000 chips are pin compatible, just replace the dw1000 chip on the loco deck with a dw3000 chip then you will get a brand new [adhoc deck](https://github.com/SEU-NetSI/crazyflie-firmware/assets/42486690/f0ec9681-9aff-4e16-8ad3-3da7a20e6b60).

## Buiding and Flashing

The compilation requirements and basic steps are the same as the official Crazyflie firmware, if you are new to Crazyflie, please refer to the [official documentation](https://github.com/bitcraze/crazyflie-firmware/blob/master/docs/building-and-flashing/build.md) for more details.

There are three default Kbuild configuration templates in the `configs` folder corresponding to three different modes, `adhoc_defconfig` for uart1, `adhoc_alt_defconfig` for IO2 / IO3 and `adhoc_uart2_defconfig` for UART2. The pin definition of UART1 and IO2 / IO3 modes are identical to loco deck, if you want to use the UART2 mode you need to use our customised expansion board circuit.

You can make custom build options on top of our Kbuild template：

```Shell
make adhoc_defconfig
```

or enable the corresponding configuration option via `make menuconfig -> Expansion deck configuration -> Support the Adhoc deck`.

> The UART1 mode is the default when the `adhoc deck alternative IRQ and RESET pins` and `adhoc deck use UART2 (TX2, RX2) pins` option are unchecked.

Then compile and flash the firmware:

```Shell
# Compile
make clean
make -j
# Flash
make cload
```

## Ranging and Messaging

The persistent parameter `MY_UWB_ADDRESS` is used as the identity of each drone, so before ranging and messaging, we should manually setting it up using the [interactive python script](https://github.com/SEU-NetSI/crazyflie-firmware/blob/master/set_uwb_address.py) or via `cfclient -> Parameters -> ADHOC -> MY_UWB_ADDRESS`.

### Ranging

Ranging is enabled out of the box, so after setting `MY_UWB_ADDRESS` for each drone, we can observe the computed distance through the Plotter provided by cfclient. The log parameter can be found at the log group named `Ranging`.

You can also programmatically get distance informations:

```C
// swarm_ranging.h
int16_t getDistance(uint16_t neighborAddress);
```

### Messaging

Here are the interfaces to send and receive packets via UWB.

```C
// adhocdeck.h
int uwbSendPacket(UWB_Packet_t *packet);
int uwbSendPacketBlock(UWB_Packet_t *packet);
int uwbReceivePacket(MESSAGE_TYPE type, UWB_Packet_t *packet);
int uwbReceivePacketBlock(MESSAGE_TYPE type, UWB_Packet_t *packet);
int uwbReceivePacketWait(MESSAGE_TYPE type, UWB_Packet_t *packet, int wait);
void uwbRegisterListener(UWB_Message_Listener_t *listener);
```

To send a packet, you only need to call the corresponding packet sending interface:

```C
// Non-blocking
int uwbSendPacket(UWB_Packet_t *packet);
// Blocking
int uwbSendPacketBlock(UWB_Packet_t *packet);
```

To receive packet you first need to specify what type of message you want to receive, there are three pre-defined message types out of the box:

```C
typedef enum {
  RANGING = 0, // Ranging message
  FLOODING = 1, // Flooding message
  DATA = 2, // Routing message
  MESSAGE_TYPE_COUNT, // don't use it
} MESSAGE_TYPE;
```

If you want to define a custom message type, simply add the message type definition (for example, CUSTOM_MESSAGE_TYPE) to the above enumeration structure.

Then create a message reception queue, register a listener for the corresponding message type by calling `uwbRegisterListener`.

```C
static QueueHandle_t rxQueue;
void exampleTaskInit() {
  // init reception queue
  rxQueue = xQueueCreate(EXAMPLE_RX_QUEUE_SIZE, EXAMPLE_RX_QUEUE_ITEM_SIZE);
  
  // init listener
  UWB_Message_Listener_t listener;
  listener.type = CUSTOM_MESSAGE_TYPE;
  listener.rxQueue = rxQueue;
  listener.rxCb = exampleRxCallback;
  listener.txCb = exampleTxCallback;
  
  // register listener
  uwbRegisterListener(&listener);
    
  // ........
}
```

> Each message type can only bind to one listener, registered listeners of the same message type will overwrite each other in the order they are registered.

Lastly, to receive and process messages, we need to create a message receiving task:

```C
static void uwbExampleRxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t rxPacketCache;
  
  while (true) {
    if (uwbReceivePacketBlock(CUSTOM_MESSAGE_TYPE, &rxPacketCache)) {
      // receive and process CUSTOM_MESSAGE_TYPE message.
    }
    // ........
  }
}
```

## BibTex

If you find this repository helpful for your work, please kindly cite the following paper:

```
@article{shan2021ultra,
  title={Ultra-Wideband Swarm Ranging},
  author={Shan, Feng and Zeng, Jiaxin and Li, Zengbao and Luo, Junzhou and Wu, Weiwei},
  booktitle={IEEE INFOCOM 2021-IEEE Conference on Computer Communications},
  year={2021},
  organization={IEEE}
}

@article{shan2022ultra,
  title={Ultra-Wideband Swarm Ranging Protocol for Dynamic and Dense Networks},
  author={Shan, Feng and Huo, Haodong and Zeng, Jiaxin and Li, Zengbao and Wu, Weiwei and Luo, Junzhou},
  journal={IEEE/ACM Transactions on Networking},
  year={2022},
  publisher={IEEE}
}
```

If you have any question, please issue the project or [email](mailto:shanfeng@seu.edu.cn) us and we will reply you soon.
