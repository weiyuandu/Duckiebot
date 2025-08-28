#!/usr/bin/env python3

import rospy
import time

def list_services():
    """List all available ROS services"""
    rospy.init_node('service_lister', anonymous=True)
    
    print("Waiting for services to start...")
    time.sleep(5)
    
    print("\nListing all available services:")
    try:
        services = rospy.get_published_topics()
        print("Topics:", services)
    except:
        pass
    
    try:
        import rosservice
        services = rosservice.get_service_list()
        print("\nServices:")
        for service in services:
            print(f"  - {service}")
            
        # Look specifically for LED-related services
        led_services = [s for s in services if 'led' in s.lower() or 'light' in s.lower()]
        print(f"\nLED/Light related services:")
        for service in led_services:
            print(f"  🔴 {service}")
            
    except Exception as e:
        print(f"Error listing services: {e}")

if __name__ == '__main__':
    list_services()
