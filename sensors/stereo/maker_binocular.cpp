#include "maker_binocular.h"
#include <iostream>
#include <stdio.h>
#include <boost/concept_check.hpp>

makerbinocular::makerbinocular()
{
    init();
}

makerbinocular::~makerbinocular()
{

    libusb_free_device_list(devs, 1);
    libusb_close(dev_handle);

    delete datain;
}


void makerbinocular::init()
{
    initialized = false;
    // init required data
    buffer_size = 640 * 480 * 2 + 2048;
    datain = new u8[buffer_size];
    has_new_frame = false;


    current_image_time = 0;
    current_imu_time = 0;
    time_elapsed =  0;

    int r;
    int err;
    ssize_t cnt;

    // initialize a library session
    r = libusb_init(&contex);

    if (r < 0)
    {
        std::cout <<  "Init error!" <<  std::endl;
    }

    // set verbosity level to 3
    libusb_set_debug(contex,  3);

    cnt = libusb_get_device_list(contex,  &devs);

    if (cnt < 0)
    {
        std::cout <<  "Get devices error" <<  std::endl;
    }
    else
    {
        std::cout <<  "Get " <<  cnt <<  " devices in total" <<  std::endl;
    }

    // found cypress usb device
    bool idVendorAndProductfound =  false;
    for (int i = 0; i < cnt; i++)
    {
        device = devs[i];
        err = libusb_get_device_descriptor(device, &desc);

        if (err < 0)
        {
            std::cout <<  "failed to get desc" <<  std::endl;
            return;
        }

        if (desc.idVendor ==  0x04b4 && desc.idProduct ==  0x1005)
        {
            std::cout <<  "============================================" << std::endl;
            printf("Found cypress usb device: idVendor 0x%04x idProduct: 0x%04x\r\n", desc.idVendor,desc.idProduct);
            std::cout <<  "============================================" <<  std::endl;
            idVendorAndProductfound = true;
            break;
        }
    }

    if (idVendorAndProductfound ==  false)
    {
        std::cout <<  "============================================" <<  std::endl;
        std::cout <<  "Error: Can not found the device, please check the idVendor and idProduct!" <<  std::endl;
        std::cout <<  "============================================" <<  std::endl;
        return ;
    }
    // get cypress usb config desc
    libusb_get_config_descriptor(device,  0,  &config);

    //

    if ((int)config->bNumInterfaces > 1)
    std::cout <<  "too many interfaces" <<  std::endl;

    const struct libusb_interface * inter;
    const struct libusb_interface_descriptor * interdesc;
    const struct libusb_endpoint_descriptor * epdesc;

    inter = &config->interface[0];
    interdesc = &inter->altsetting[0];

    if ((int) interdesc->bNumEndpoints > 2)
    std::cout <<  "too many endpoints" <<  std::endl;

    for (int j = 0; j < interdesc->bNumEndpoints; j++)
    {
        epdesc = &interdesc->endpoint[j];

        if ((epdesc->bEndpointAddress) & 0x80)
        {
            bulk_ep_in = epdesc->bEndpointAddress;
            printf("Hints: Get Built in endpoint: 0x%02x\n" ,  epdesc->bEndpointAddress);
            printf("Max packetsize is %d \n",  epdesc->wMaxPacketSize);
        }
    }

    err = libusb_open(device, &dev_handle);

    if (err < 0)
    {
        printf("open device failed\n");
        libusb_free_device_list(devs, 1);
        libusb_close(dev_handle);
        return ;
    }

    if (libusb_kernel_driver_active(dev_handle,  0) ==  1)
    {
        printf("Kernel Driver Active\n" );
        if (libusb_detach_kernel_driver(dev_handle,  0 ) ==  0)
        printf("Kernal Driver Detached\n");
    }

    err = libusb_claim_interface(dev_handle,  0);
    if (err < 0)
    {
        printf("can not claim interface");
        libusb_free_device_list(devs, 1);
        libusb_close(dev_handle);
        return;
    }

    initialized = true;
}

void makerbinocular::get_imu_data(float acc[3], float gyro[3])
{
    for (int i = 0; i < 3; i++)
    {
        acc[i] = acc_raw[i];
        gyro[i] = gyro_raw[i];
    }
}


void makerbinocular::get_frame(cv::Mat &left_image, cv::Mat &right_image)
{
    time_elapsed = 0;
    has_new_frame = false;

    if (((left_image.rows !=  480) |  (left_image.cols !=  640) |  (right_image.rows !=  480) |  right_image.cols) !=  640)
    {
        std::cout <<  left_image.rows <<  left_image.cols <<  std::endl;
        std::cout <<  "Error: the image size should be: 640 x 480" <<  std::endl;
    }

    int transferd;
    unsigned char * pcS = (u8*) (datain + 32);
    int error = libusb_bulk_transfer(dev_handle, bulk_ep_in, datain, buffer_size, &transferd, 1000);

    //std::cout <<  transferd <<  std::endl;

    if (transferd ==  0)
    {
        std::cout <<  "============================================" <<  std::endl;
        std::cout <<  "Warning: No data received ! Please check the buld endpoint address" <<  std::endl;
        std::cout <<  "============================================" <<  std::endl;
    }

    if (error == 0) 
    {
        // frame header
        //std::cout << (int)(*datain) << " " << (int)(*(datain+1)) << " " << (int)(*(datain+2)) << " " << (int)(*(datain+3)) << std::endl;     
        if ((((*datain) ==  0xfe) & (*(datain+1) ==  0x01) & (*(datain+2) ==  0xfe) & (*(datain + 3) ==  0x01)) != 1)
        {
            //std::cout <<  "get frame header" <<  std::endl;
            return;
        }

        int time_stamp = *(datain + 8) | *(datain + 9) <<  8;
        time_elapsed = 1.0 * time_stamp * 256 / 108;
//        std::cout <<  time_elapsed <<  std::endl;

        current_image_time = current_image_time + time_elapsed;
        current_imu_time = current_imu_time + time_elapsed;

        int raw_acc_x = (short) (datain[12] | datain[13] << 8);
        int raw_acc_y = (short) (datain[14] | datain[15] << 8);
        int raw_acc_z = (short) (datain[16] | datain[17] << 8);

        int raw_gyro_x = (short) (datain[18] | datain[19] << 8);
        int raw_gyro_y = (short) (datain[20] | datain[21] << 8);
        int raw_gyro_z = (short) (datain[22] | datain[23] << 8);

        int raw_temprature = (short) (datain[24] | datain[25] << 8);

        acc_raw[0] = raw_acc_x * 1.0 / 16384 * 9.8;
        acc_raw[1] = raw_acc_y * 1.0 / 16384 * 9.8;
        acc_raw[2] = raw_acc_z * 1.0 / 16384 * 9.8;

        gyro_raw[0] = raw_gyro_x * 1.0 / 16.4;
        gyro_raw[1] = raw_gyro_y * 1.0 / 16.4;
        gyro_raw[2] = raw_gyro_z * 1.0 / 16.4;

        //std::cout <<  acc_raw[0] << " " <<  acc_raw[1] <<  " "<< acc_raw[2] <<  std::endl;
        //std::cout <<  gyro_raw[0] << " " <<  gyro_raw[1] <<  " "<< gyro_raw[2] <<  std::endl;

        /*
        else
        {
            std::cout << "========================================" <<  std::endl;
            std::cout <<  "Warning: Frame header error!" <<  std::endl;
            std::cout << "=======================================" <<  std::endl;
            usleep(30000);
            return;
        }
        */

        int cnt_y,  cnt_x;
        for (cnt_y  = 0; cnt_y < 480; cnt_y++)
        {
            for (cnt_x = 0; cnt_x < 640; cnt_x++)
            {
                // left image
                left_image.at<uchar>(479 - cnt_y, cnt_x) = *(pcS + cnt_y * 1280 + cnt_x * 2);

                // right image
                right_image.at<uchar>(479 - cnt_y, cnt_x) = *(pcS + cnt_y * 1280 + cnt_x * 2 + 1);
            }
        }

    }

    has_new_frame = true;
}
