#!/usr/bin/env python3
"""
OLED Driver for 128x32 SSD1306 Display via I2C
Designed for Jetson Orin Nano with Waveshare JetRacer
"""

import time
import smbus2
from PIL import Image, ImageDraw, ImageFont
import logging

# SSD1306 Commands
SSD1306_DISPLAYOFF = 0xAE
SSD1306_DISPLAYON = 0xAF
SSD1306_SETDISPLAYCLOCKDIV = 0xD5
SSD1306_SETMULTIPLEX = 0xA8
SSD1306_SETDISPLAYOFFSET = 0xD3
SSD1306_SETSTARTLINE = 0x40
SSD1306_CHARGEPUMP = 0x8D
SSD1306_MEMORYMODE = 0x20
SSD1306_SEGREMAP = 0xA1
SSD1306_COMSCANDEC = 0xC8
SSD1306_SETCOMPINS = 0xDA
SSD1306_SETCONTRAST = 0x81
SSD1306_SETPRECHARGE = 0xD9
SSD1306_SETVCOMDETECT = 0xDB
SSD1306_NORMALDISPLAY = 0xA6
SSD1306_COLUMNADDR = 0x21
SSD1306_PAGEADDR = 0x22


class OLEDDriver:
    """Driver class for SSD1306 128x32 OLED display via I2C"""

    def __init__(self, i2c_bus=7, i2c_address=0x3C, width=128, height=32):
        """
        Initialize OLED display

        Args:
            i2c_bus (int): I2C bus number (default: 7 for Jetson Orin Nano GPIO header)
            i2c_address (int): I2C address of OLED (default: 0x3C)
            width (int): Display width in pixels (default: 128)
            height (int): Display height in pixels (default: 32)
        """
        self.width = width
        self.height = height
        self.i2c_address = i2c_address
        self.pages = height // 8

        # Initialize I2C bus
        try:
            self.bus = smbus2.SMBus(i2c_bus)
            logging.info(f"I2C bus {i2c_bus} initialized successfully")
        except Exception as e:
            logging.error(f"Failed to initialize I2C bus {i2c_bus}: {e}")
            raise

        # Create image buffer
        self.image = Image.new('1', (width, height))
        self.draw = ImageDraw.Draw(self.image)

        # Initialize display
        self._init_display()
        
    def _init_display(self):
        """Initialize the OLED display with proper settings"""
        init_sequence = [
            SSD1306_DISPLAYOFF,
            SSD1306_SETDISPLAYCLOCKDIV, 0x80,
            SSD1306_SETMULTIPLEX, 0x1F,  # 32-1
            SSD1306_SETDISPLAYOFFSET, 0x00,
            SSD1306_SETSTARTLINE | 0x00,
            SSD1306_CHARGEPUMP, 0x14,
            SSD1306_MEMORYMODE, 0x00,
            SSD1306_SEGREMAP | 0x01,
            SSD1306_COMSCANDEC,
            SSD1306_SETCOMPINS, 0x02,
            SSD1306_SETCONTRAST, 0x8F,
            SSD1306_SETPRECHARGE, 0xF1,
            SSD1306_SETVCOMDETECT, 0x40,
            SSD1306_NORMALDISPLAY,
            SSD1306_DISPLAYON
        ]
        
        try:
            for cmd in init_sequence:
                self._write_command(cmd)
            logging.info("OLED display initialized successfully")
            self.clear()
            self.display()
        except Exception as e:
            logging.error(f"Failed to initialize OLED display: {e}")
            raise
    
    def _write_command(self, cmd):
        """Write command to OLED display"""
        try:
            self.bus.write_byte_data(self.i2c_address, 0x00, cmd)
        except Exception as e:
            logging.error(f"Failed to write command 0x{cmd:02X}: {e}")
            raise

    def _write_data(self, data):
        """Write data to OLED display"""
        try:
            self.bus.write_byte_data(self.i2c_address, 0x40, data)
        except Exception as e:
            logging.error(f"Failed to write data: {e}")
            raise
    
    def clear(self):
        """Clear the display buffer"""
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)
    
    def display(self):
        """Send the image buffer to the OLED display"""
        try:
            # Set column address range
            self._write_command(SSD1306_COLUMNADDR)
            self._write_command(0)  # Column start address
            self._write_command(self.width - 1)  # Column end address

            # Set page address range
            self._write_command(SSD1306_PAGEADDR)
            self._write_command(0)  # Page start address
            self._write_command(self.pages - 1)  # Page end address

            # Convert image to bytes and send to display
            image_data = list(self.image.getdata())
            buffer = []

            for page in range(self.pages):
                for x in range(self.width):
                    byte = 0
                    for bit in range(8):
                        y = page * 8 + bit
                        if y < self.height:
                            pixel = image_data[y * self.width + x]
                            if pixel:
                                byte |= (1 << bit)
                    buffer.append(byte)

            # Send buffer to display
            for byte in buffer:
                self._write_data(byte)

        except Exception as e:
            logging.error(f"Failed to update display: {e}")
            raise
    
    def text(self, text, x=0, y=0, font_size=12):
        """
        Draw text on the display
        
        Args:
            text (str): Text to display
            x (int): X coordinate
            y (int): Y coordinate
            font_size (int): Font size
        """
        try:
            # Use default font for now
            self.draw.text((x, y), text, font=None, fill=1)
        except Exception as e:
            logging.error(f"Failed to draw text '{text}': {e}")
    
    def line(self, x0, y0, x1, y1):
        """Draw a line on the display"""
        self.draw.line([(x0, y0), (x1, y1)], fill=1)
    
    def rectangle(self, x0, y0, x1, y1, fill=False):
        """Draw a rectangle on the display"""
        if fill:
            self.draw.rectangle([(x0, y0), (x1, y1)], outline=1, fill=1)
        else:
            self.draw.rectangle([(x0, y0), (x1, y1)], outline=1, fill=0)
    
    def close(self):
        """Close the I2C connection"""
        try:
            self.clear()
            self.display()
            self.bus.close()
            logging.info("OLED driver closed successfully")
        except Exception as e:
            logging.error(f"Error closing OLED driver: {e}")


if __name__ == "__main__":
    # Test the OLED driver
    logging.basicConfig(level=logging.INFO)
    
    try:
        oled = OLEDDriver()
        oled.clear()
        oled.text("OLED Test", 0, 0)
        oled.text("128x32 Display", 0, 12)
        oled.display()
        
        time.sleep(3)
        
        oled.clear()
        oled.text("Driver Working!", 0, 10)
        oled.display()
        
        time.sleep(2)
        oled.close()
        
    except Exception as e:
        logging.error(f"OLED test failed: {e}")
