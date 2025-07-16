#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import mysql.connector
from multi_map_manager.srv import GetWormhole

class WormholeService(Node):
    def __init__(self):
        super().__init__('wormhole_service')
        self.srv = self.create_service(GetWormhole, 'get_wormhole', self.wormhole_callback)
        
        # MySQL connection
        self.conn = mysql.connector.connect(
            host="localhost",
            user="root",
            password="root",
            database="wormhole_loc"
        )
        
    def wormhole_callback(self, request, response):
        try:
            cursor = self.conn.cursor(dictionary=True)
            query = """SELECT * FROM wormholes 
                      WHERE from_room=%s AND to_room=%s LIMIT 1"""
            cursor.execute(query, (request.from_room, request.to_room))
            result = cursor.fetchone()
            
            if not result:
                response.success = False
                return response
            
            # Calculate entry pose (centroid)
            x = (result['x1'] + result['x2'] + result['x3'] + result['x4']) / 4
            y = (result['y1'] + result['y2'] + result['y3'] + result['y4']) / 4
            
            response.entry_pose.header.frame_id = "map"
            response.entry_pose.pose.position.x = x
            response.entry_pose.pose.position.y = y
            response.entry_pose.pose.orientation.w = 1.0
            
            # Calculate exit pose
            response.exit_x = (result['x3'] + result['x4']) / 2
            response.exit_y = (result['y3'] + result['y4']) / 2
            response.exit_yaw = 0.0
            response.success = True
            
        except Exception as e:
            self.get_logger().error(f"MySQL error: {str(e)}")
            response.success = False
            
        return response

def main():
    rclpy.init()
    node = WormholeService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
