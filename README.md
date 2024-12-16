# DexHand021 CPP Linux SDK

A comprehensive CPP development interfaces for robotic hand manipulation of Linux. This project provides a minimal stack for controling dexterous robotic hands, with low-level protocol interfaces.

## Prerequisition and prepare

1. A DexHand021 dexterous hand product
2. A ZLG USBCANFD_200U device, for communication between dexhand021 product and your application.
3. Ubuntu 18 or higher version
4. Connect your DexHand021 product to your PC with ZLG USBCANFD_200U, and make sure the DexHand021 product is power on.

## How to use

### 1. Create a Dexterous_hand instance of your DexHand021 product
To manipulate a DexHand021 product with this SDK, you need to create a Dexterous_hand instance first. Class Dexterous_hand is defined in header file Dexterous_hands.h.

### 2. Start the Dexterous_hand instance
Once the dexhand instance is created, you can simply call start() method to eslablish a connection between your program and DexHand021 product.

### 3. Control your DexHand021 product
Once the connection is established, now you can call the *_Control_* interfaces to control the hand, or use other interfaces to get/set the information of the hand.

### 4. Close the instance
Once you finish your works, please remeber to close the instance by calling stop().

Refer to our SDK references for more details of how to use. To download the reference, please go to our website https://www.dex-robot.com/downloadCenter/detail

