from MassLib import mk_process
from PIL import Image, ImageDraw, ImageFont

def simpleTextImage(message, header=''):
    img = TextImage('HUD.png')
    num_message=0
    msgs = []
    while t_width(img.font, message)>900:
        for i in range(len(message.split())):
            if t_width(img.font, ' '.join(message.split()[:i+1])) > 900:
                msgs.append(' '.join(message.split()[:i]))
                message = ' '.join(message.split()[i:])
                print msgs, message
    print msgs
    if message != '':
        msgs.append(message)
    for i, msg in enumerate(msgs):
        img.add_text(i+1, msg)
    img.add_text(0, header)

    img.save_image()
    img.send_image()

def t_width(font, msg):
    return font.getsize(msg)[0]

class TextImage():
    def __init__(self, filename):
        self.img_size = (1024, 600)
        self.font_size = 55 
        self.filename = filename
        self.font_ttf = 'tester/testbody/resources/FreeSerif.ttf'
        
        self.image = Image.new('RGB', self.img_size)
        self.draw = ImageDraw.Draw(self.image)
        self.font = ImageFont.truetype(self.font_ttf, self.font_size)

        self.centerX = self.img_size[0]/2
        self.centerY = self.img_size[1]/2

        self.text_height = self.font.getsize('W')[1]
        self.between_text = 10

    def add_text(self, y_pos, text):
        text_size = self.font.getsize(text)[0]
        x_pos = self.centerX - text_size/2        
        y_pos = self.centerY + (self.text_height * (y_pos-1.5) + (self.between_text * (y_pos-1)))
        self.draw.text((x_pos, y_pos), text, font=self.font)

    def save_image(self):
        self.image.save(open(self.filename, 'wb'), 'PNG')
    
    def display(self):
        mk_process('eog %s' % self.filename)

    def send_image(self):
        mk_process('rosrun head_control xdisplay_image.py -f %s' % self.filename)
