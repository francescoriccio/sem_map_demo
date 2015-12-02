#!/usr/bin/env python
#
# A simple SPIDER image viewer.
# Image (Python Imaging Library) used to convert SPIDER image,
# Tkinter used for display.

import Tkinter as tk
import tkMessageBox
import ConfigParser
from Tkinter import *
from ttk import *
import Image, ImageTk, tkFileDialog
import numpy as np
import sys, time, os, glob, shutil
from math import atan2, degrees, radians

extension_list = ['png','ppm','jpg','gif','pgm']
maps_image_folder_name = "img/maps"
robots_image_folder_name = "img/robots"
maps_folder = "../maps/"
robot_launch_files_folder = "../launch/robots/"

def norm360(a):
    while (a>360):
      a = a-360
    while (a<=0):
      a = a+360
    return a

class DIP(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent) 
        self.parent = parent        
        self.initUI()

    def centerWindow(self):
        w = 290
        h = 150
        sw = self.parent.winfo_screenwidth()
        sh = self.parent.winfo_screenheight()
        x = (sw - w)/2
        y = (sh - h)/2
        self.parent.geometry('%dx%d+%d+%d' % (w, h, x, y))
        
    def initUI(self):
        self.mouseclickx = -1
        self.mouseclicky = -1
        self.home_pos_x = None
        self.robot_pos_x = None
        
        self.tryLoadOldConfig()
        self.loadPorts()
        
        self.init_images_in_folder()
        self.parent.title("living_at_diag Demo Launcher")
        self.style = Style()
        self.style.theme_use("alt")
        self.parent.resizable(width=FALSE, height=FALSE)
        self.pack(fill=BOTH, expand=1)
        self.centerWindow()

        self.columnconfigure(1, weight=1)
        self.columnconfigure(3, pad=7)
        self.rowconfigure(3, weight=1)
        self.rowconfigure(5, pad=7)
        
        lbl = Label(self, text="Map")
        lbl.grid(sticky=W, row = 0, column= 1, pady=4, padx=5)
        
        lbl = Label(self, text="Robot")
        lbl.grid(sticky=W, row = 0, column= 0, pady=4, padx=5)
        
        self.map_images_list = sorted(self.map_images_list)
        self.map_name_list = sorted(self.map_name_list)
        
        self.setMapImage(self.map_images_list[0])
        try:
          self.setRobotImage(str(self.oldConfigs["robotname"]))
        except:
          self.setRobotImage(self.robot_name_list[0])
        
        self.map_ddm = StringVar(self)
        self.map_ddm.set(self.map_name_list[0])
        tk.OptionMenu(self,self.map_ddm, *self.map_name_list,command=self.update_map_image).grid(sticky=W, row=10, column=1, pady=4, padx=5)
        
        self.robot_ddm = StringVar(self)
        
        try:
          self.robot_ddm.set(str(self.oldConfigs["robotname"]))
        except:
          self.robot_ddm.set(self.robot_name_list[0])
        
        tk.OptionMenu(self,self.robot_ddm, *self.robot_name_list,command=self.update_robot_image).grid(sticky=W, row=10, column=0, pady=4, padx=5)
        
        menubar = tk.Menu(self.parent)
        self.parent.config(menu = menubar)
        
        self.frame2 = Frame(self)
        self.frame2.grid(sticky=W, row=2, column=2, pady=1, padx=1)
        
        self.kb = IntVar()
        try:
          self.kb.set(str(self.oldConfigs["kbenabled"]))
        except:
          self.kb.set('0')
        
        kb_check = Checkbutton(self.frame2, text="KB", variable=self.kb)
        kb_check.grid(sticky=W, row=0, column=0, pady=1, padx=1)
        
        self.dd = IntVar()
        try:
          self.dd.set(str(self.oldConfigs["dotdetectorenabled"]))
        except:
          self.dd.set('0')
        dd_check = Checkbutton(self.frame2, text="Dot Detector", variable=self.dd)
        dd_check.grid(sticky=W, row=1, column=0, pady=1, padx=1)
        
        self.combine_pnp = IntVar()
        try:
          self.combine_pnp.set(str(self.oldConfigs["combine_pnpenabled"]))
        except:
          self.combine_pnp.set('0')
        combine_pnp_check = Checkbutton(self.frame2, text="Learn Tasks", variable=self.combine_pnp)
        combine_pnp_check.grid(sticky=W, row=2, column=0, pady=1, padx=1)
        
        self.logger = IntVar()
        try:
          self.logger.set(str(self.oldConfigs["loggerenabled"]))
        except:
          self.logger.set('0')
        logger_check = Checkbutton(self.frame2, text="Logs", variable=self.logger)
        logger_check.grid(sticky=W, row=3, column=0, pady=1, padx=1)
        
        #draw buttons
        self.frame1 = Frame(self)
        self.frame1.grid(sticky=W, row=1, column=2, pady=1, padx=1)
        
        setPoseButton = Button(self.frame1, text="Set Init Pose",command=self.show_init_pos_pic)
        setPoseButton.grid(sticky=W, row=0, column=0, pady=5, padx=1)
        
        setPoseButton = Button(self.frame1, text="Set Home Pose",command=self.show_home_pos_pic)
        setPoseButton.grid(sticky=W, row=1, column=0, pady=5, padx=1)
        
        launchButton = Button(self, text="Launch Demo",command=self.launch_script)
        launchButton.grid(sticky=W, row=10, column=2, pady=1, padx=1)
        
        launchButton = Button(self, text="Kill Demo",command=self.kill_demo)
        launchButton.grid(sticky=W, row=11, column=2, pady=1, padx=1)
        
        #Open Image Menu
        fileMenu = tk.Menu(menubar)
        fileMenu.add_command(label = "Modify Devices Ports", command = self.show_ports_window)
        fileMenu.add_command(label = "Quit", command = self.quit)
        menubar.add_cascade(label = "File", menu = fileMenu)

    def init_images_in_folder(self):
        self.map_name_list = []
        self.robot_name_list = []
        if os.path.exists(maps_image_folder_name):
          shutil.rmtree(maps_image_folder_name)
        os.makedirs(maps_image_folder_name)
        if(os.path.exists(maps_image_folder_name)):
          try:
            for extension in extension_list:
              for files in glob.glob(maps_folder+"*."+extension):
                shutil.copy2(files, maps_image_folder_name)
                self.map_name_list.append(files.split("/")[len(files.split("/"))-1].split(".")[0])
            self.map_images_list = glob.glob(maps_image_folder_name+"/*")
          except:
            print "\033[91mAn error occurred while retrieving the maps\033[0m"
        
          try:
            for files in glob.glob(robot_launch_files_folder+"*"):
              self.robot_name_list.append((files.split("/")[len(files.split("/"))-1].split(".")[0]))
            
          except:
            print "\033[92mAn error occurred while retrieving robot names\033[0m"

    def setMapImage(self, filename):
        self.img = Image.open(filename)
        self.img_res = self.img.resize((250, 250), Image.ANTIALIAS)
        self.tkimage_res = ImageTk.PhotoImage(self.img_res)
        self.map_image = Label(self, image=self.tkimage_res)
        self.map_image.grid(row=1, column=1, columnspan=1, rowspan=9, padx=5, sticky=E+W+S+N)
        
    def update_map_image(self, value):
        map_name = glob.glob(maps_image_folder_name+"/"+value+"*")[0]
        self.img = Image.open(map_name)
        self.img_res = self.img.resize((250, 250), Image.ANTIALIAS)
        self.tkimage_res = ImageTk.PhotoImage(self.img_res)
        self.map_image.configure(image = self.tkimage_res)
    
    def setRobotImage(self, filename):
        try:
          filename = glob.glob(robots_image_folder_name+"/"+filename+"*")[0]
        except:
          filename = robots_image_folder_name+"/default.png" 
        self.img2 = Image.open(filename)
        self.img2 = self.img2.resize((250, 250), Image.ANTIALIAS)
        self.tkimage_res2 = ImageTk.PhotoImage(self.img2)
        self.robot_image = Label(self, image=self.tkimage_res2)
        self.robot_image.grid(row=1, column=0, columnspan=1, rowspan=9, padx=5, sticky=E+W+S+N)
        
    def update_robot_image(self, value):
        try:
          robot_image_name = glob.glob(robots_image_folder_name+"/"+value+"*")[0]
        except:
          robot_image_name = robots_image_folder_name+"/default.png"
          
        self.img2 = Image.open(robot_image_name)
        self.img2 = self.img2.resize((250, 250), Image.ANTIALIAS)
        self.tkimage_res2 = ImageTk.PhotoImage(self.img2)
        self.robot_image.configure(image = self.tkimage_res2)
        
    def launch_script(self):
        if self.robot_pos_x != None and self.home_pos_x != None:
          self.saveConfigFile()
            
          terminal_command = "roslaunch living_at_diag launcher.launch robot_name:=%s map_name:=%s init_x:=%s init_y:=%s init_a:=%s robotPort:=%s laserPort:=%s  laserPort2:=%s joyPort:=%s home_x:=%s home_y:=%s home_th:=%s "%(self.robot_ddm.get(),self.map_ddm.get(), self.robot_pos_x, self.robot_pos_y, radians(self.robot_angle), self.robotPort, self.laserPort, self.laserPort2, self.joyPort, self.home_pos_x, self.home_pos_y, self.home_angle)
          if self.robot_ddm.get() != "robot0":    
            terminal_command += "simulation:=false &"
          else :
            terminal_command += "simulation:=true &"
            
          os.system(terminal_command)
         
          if self.kb.get() == 1:
            time.sleep(3)
            os.system("xterm -e roslaunch sapienzbot_reasoning sapienzBot.launch &")
            time.sleep(10)
            os.system("xterm -e roslaunch semantic_map_extraction semantic_map_extraction.launch map_name:=%s &"%(self.map_ddm.get()))
          
          if self.dd.get() == 1:
            time.sleep(3)
            if(self.robot_ddm.get() == "turtlebot_2"):
              os.system("xterm -e roslaunch dot_detector dotdetector.launch robot_name:=%s laser_topic:=scan &"%self.robot_ddm.get())
            elif (self.robot_ddm.get() == "robot0" or self.robot_ddm.get() == "bigRodent"):
              #os.system("xterm -e roslaunch dis_robots %s_openni.launch &"%self.robot_ddm.get())
	      #os.system("xterm -e roslaunch openni2_launch openni2.launch &")
              time.sleep(3)
              os.system("xterm -e roslaunch dot_detector dotdetector.launch robot_name:=%s laser_topic:=scan &"%self.robot_ddm.get())
            else:
              os.system("xterm -e roslaunch dot_detector dotdetector.launch robot_name:=%s &"%self.robot_ddm.get())
            
          if self.combine_pnp.get() == 1:
            os.system("xterm -e roslaunch combine_pnp combine_pnp.launch &")
            
          if self.logger.get() == 1:
            os.system("xterm -e roslaunch logger logger.launch &")
            
        elif self.robot_pos_x == None and self.home_pos_x == None:
          if tkMessageBox.askyesno("Print", "Initial and Home Positions Not Set. Start the robot with the last ones used?"):
            try:
              self.robot_pos_x = float(self.oldConfigs["init_x"])
              self.robot_pos_y = float(self.oldConfigs["init_y"])
              self.robot_angle = float(self.oldConfigs["init_a"])
              self.home_pos_x = float(self.oldConfigs["home_x"])
              self.home_pos_y = float(self.oldConfigs["home_y"])
              self.home_angle = float(self.oldConfigs["home_th"])
              self.launch_script()
            except:
              self.robot_pos_x = None
              self.home_pos_x = None
              tkMessageBox.showerror("Error", "Could not retrieve last configurations used")
            
        elif self.robot_pos_x == None:
          if tkMessageBox.askyesno("Print", "Initial Position Not Set. Start the robot with the last one used?"):
            try:
              self.robot_pos_x = float(self.oldConfigs["init_x"])
              self.robot_pos_y = float(self.oldConfigs["init_y"])
              self.robot_angle = float(self.oldConfigs["init_a"])
              self.launch_script()
            except:
              self.robot_pos_x = None
              tkMessageBox.showerror("Error", "Could not retrieve last configurations used")
        
        elif self.home_pos_x == None:
          if tkMessageBox.askyesno("Print", "Home Position Not Set. Start the robot with the last one used?"):
            try:
              self.home_pos_x = float(self.oldConfigs["home_x"])
              self.home_pos_y = float(self.oldConfigs["home_y"])
              self.home_angle = float(self.oldConfigs["home_th"])
              self.launch_script()
            except:
              self.home_pos_x = None
              tkMessageBox.showerror("Error", "Could not retrieve last configurations used")
            
    def drawRobotIcon(self,event):  
        self.oldeventx = event.x
        self.oldeventy = event.y
        self.mouseclickx = event.x + self.xscrollbar.get()[0]*self.imgPose.width() - 10
        self.mouseclicky = event.y + self.yscrollbar.get()[0]*self.imgPose.height() - 10
        self.robot_angle = 90
        self.converted_mouseclicky= -(self.mouseclicky - self.imgPose.height())
        
        self.canvas.delete(self.riiCanvas)
        robot_icon_name = robots_image_folder_name+"/robot_icon.png"
        self.ri = Image.open(robot_icon_name)
        self.ri = self.ri.resize((int(self.imgPose.width()/50), int(self.imgPose.width()/50)), Image.ANTIALIAS)
        self.rii = ImageTk.PhotoImage(self.ri.rotate(self.robot_angle-90))
        self.riiCanvas = self.canvas.create_image(self.mouseclickx, self.mouseclicky, image = self.rii, anchor = NW)
        self.set_initial_robot_pose()
        
    def drawHomeRobotIcon(self,event):  
        self.oldeventx = event.x
        self.oldeventy = event.y
        self.mouseclickx = event.x + self.xscrollbar.get()[0]*self.imgPose.width() - 10
        self.mouseclicky = event.y + self.yscrollbar.get()[0]*self.imgPose.height() - 10
        self.home_angle = 90
        self.converted_mouseclicky= -(self.mouseclicky - self.imgPose.height())
        
        self.canvas.delete(self.riiCanvas)
        robot_icon_name = robots_image_folder_name+"/robot_icon.png"
        self.ri = Image.open(robot_icon_name)
        self.ri = self.ri.resize((int(self.imgPose.width()/50), int(self.imgPose.width()/50)), Image.ANTIALIAS)
        self.rii = ImageTk.PhotoImage(self.ri.rotate(self.home_angle-90))
        self.riiCanvas = self.canvas.create_image(self.mouseclickx, self.mouseclicky, image = self.rii, anchor = NW)
        self.set_home_robot_pose()
        
    def set_initial_robot_pose(self):
        resolution_line_list = [ line for line in open(maps_folder+"/"+self.map_ddm.get()+".yaml") if 'resolution' in line]
        origin_line_list = [ line for line in open(maps_folder+"/"+self.map_ddm.get()+".yaml") if 'origin' in line]

        res = float("0."+resolution_line_list[0].split(".")[1].replace("\n",""))
        orig_x = float(origin_line_list[0].split("[")[1].split(",")[0])
        orig_y = float(origin_line_list[0].split("[")[1].split(",")[1])
        
        self.robot_pos_x = (self.mouseclickx) * res + orig_x
        self.robot_pos_y = (self.converted_mouseclicky) * res + orig_y
        self.canvas.focus_set()
        
    def set_home_robot_pose(self):
        resolution_line_list = [ line for line in open(maps_folder+"/"+self.map_ddm.get()+".yaml") if 'resolution' in line]
        origin_line_list = [ line for line in open(maps_folder+"/"+self.map_ddm.get()+".yaml") if 'origin' in line]

        res = float("0."+resolution_line_list[0].split(".")[1].replace("\n",""))
        orig_x = float(origin_line_list[0].split("[")[1].split(",")[0])
        orig_y = float(origin_line_list[0].split("[")[1].split(",")[1])
        
        self.home_pos_x = (self.mouseclickx) * res + orig_x
        self.home_pos_y = (self.converted_mouseclicky) * res + orig_y
        self.canvas.focus_set()
        
    def rotate(self, event):
        if self.mouseclickx != -1:
          self.robot_angle = norm360(-degrees(atan2(event.y-self.oldeventy,event.x-self.oldeventx)))
          self.rii = ImageTk.PhotoImage(self.ri.rotate(self.robot_angle-90))
          self.canvas.delete(self.riiCanvas)
          self.riiCanvas = self.canvas.create_image(self.mouseclickx, self.mouseclicky, image = self.rii, anchor = NW)
          
    def home_rotate(self, event):
        if self.mouseclickx != -1:
          self.home_angle = norm360(-degrees(atan2(event.y-self.oldeventy,event.x-self.oldeventx)))
          self.rii = ImageTk.PhotoImage(self.ri.rotate(self.home_angle-90))
          self.canvas.delete(self.riiCanvas)
          self.riiCanvas = self.canvas.create_image(self.mouseclickx, self.mouseclicky, image = self.rii, anchor = NW)
    
    def rotate_with_key(self, event):
        self.robot_angle = norm360(self.robot_angle+10)
        self.rii = ImageTk.PhotoImage(self.ri.rotate(self.robot_angle-90))
        self.canvas.delete(self.riiCanvas)
        self.riiCanvas = self.canvas.create_image(self.mouseclickx, self.mouseclicky, image = self.rii, anchor = NW)
    
    def home_rotate_with_key(self, event):
        self.home_angle = norm360(self.home_angle+10)
        self.rii = ImageTk.PhotoImage(self.ri.rotate(self.home_angle-90))
        self.canvas.delete(self.riiCanvas)
        self.riiCanvas = self.canvas.create_image(self.mouseclickx, self.mouseclicky, image = self.rii, anchor = NW)
        
    def show_init_pos_pic(self):
        novi = Toplevel()
        novi.title("Set Initial Pose")
        
        self.ri = None
        self.frame = Frame(novi, relief=SUNKEN)

        self.frame.grid_rowconfigure(0, weight=1)
        self.frame.grid_columnconfigure(0, weight=1)

        self.xscrollbar = Scrollbar(self.frame, orient=HORIZONTAL)
        self.xscrollbar.grid(row=2, column=0, sticky=E+W)

        self.yscrollbar = Scrollbar(self.frame)
        self.yscrollbar.grid(row=1, column=1, sticky=N+S)

        self.instructions = StringVar()
        self.instructions.set("Left Click = Position the robot. Right Click / R = Rotate the robot.")
        self.lab = Label(self.frame, textvariable=self.instructions).grid(row=0,column=0, pady=10)
        
        self.imgPose = ImageTk.PhotoImage(self.img)
        width = self.imgPose.width()
        height = self.imgPose.height()
        if (self.imgPose.width() > 640): 
          width = 640
        if (self.imgPose.height() > 480):
          height = 480
        
        self.canvas = Canvas(self.frame, width = width, height = height, bd=0, xscrollcommand=self.xscrollbar.set, yscrollcommand=self.yscrollbar.set)
        self.canvas.grid(row=1, column=0, sticky=N+S+E+W)
        
        self.canvas.create_image(0,0,image=self.imgPose, anchor="nw")

        self.xscrollbar.config(command=self.canvas.xview)
        self.yscrollbar.config(command=self.canvas.yview)
        self.canvas.config(scrollregion=self.canvas.bbox(ALL))
        self.canvas.bind("<Button-1>",self.drawRobotIcon)
        self.canvas.bind("<B3-Motion>",self.rotate)
        self.canvas.bind("r", self.rotate_with_key)
        self.riiCanvas = None 
        self.frame.pack()

    def show_ports_window(self):
        novi = Toplevel()
        novi.title("Set Devices Ports")
        self.robotString = StringVar()
        self.robotString.set("Robot USB Port Name:")
        self.labRobot = Label(novi, justify=LEFT, textvariable=self.robotString).grid(row=0,column=0, padx=3, pady = 3)
        
        self.setPortsButton = Button(novi, text="Set Ports", width=10, command=self.setPorts)
        self.setPortsButton.grid(row=5,column=1, columnspan = 1, pady=5)
        
        self.robotPortText = Entry(novi, width = 20)
        self.robotPortText.insert(0, self.robotPort)
        self.robotPortText.grid(row=0,column=1, columnspan = 2)
        
        self.laserString = StringVar()
        self.laserString.set("Laser USB Port Name:")
        self.labLaser = Label(novi, justify=LEFT, textvariable=self.laserString).grid(row=1,column=0, padx=3, pady = 3)
        
        self.laserPortText = Entry(novi, width = 20)
        self.laserPortText.insert(0, self.laserPort)
        self.laserPortText.grid(row=1,column=1, columnspan = 2)
        
        self.laserString2 = StringVar()
        self.laserString2.set("Additional Laser USB Port Name:")
        self.labLaser2 = Label(novi, justify=LEFT, textvariable=self.laserString2).grid(row=2,column=0, padx=3, pady = 3)
        
        self.laserPortText2 = Entry(novi, width = 20)
        self.laserPortText2.insert(0, self.laserPort2)
        self.laserPortText2.grid(row=2,column=1, columnspan = 2)
        
        self.joyString = StringVar()
        self.joyString.set("Joystick USB Port Name:")
        self.labJoy = Label(novi, justify=LEFT, textvariable=self.joyString).grid(row=3,column=0, padx=3, pady = 3)
        
        self.joyPortText = Entry(novi, width = 20)
        self.joyPortText.insert(0, self.joyPort)
        self.joyPortText.grid(row=3,column=1, columnspan = 2)
        
        
    def show_home_pos_pic(self):
        novi = Toplevel()
        novi.title("Set Home Pose")
        
        self.ri = None
        self.frame = Frame(novi, relief=SUNKEN)

        self.frame.grid_rowconfigure(0, weight=1)
        self.frame.grid_columnconfigure(0, weight=1)

        self.xscrollbar = Scrollbar(self.frame, orient=HORIZONTAL)
        self.xscrollbar.grid(row=2, column=0, sticky=E+W)

        self.yscrollbar = Scrollbar(self.frame)
        self.yscrollbar.grid(row=1, column=1, sticky=N+S)

        self.instructions = StringVar()
        self.instructions.set("Left Click = Position the robot. Right Click / R = Rotate the robot.")
        self.lab = Label(self.frame, textvariable=self.instructions).grid(row=0,column=0, pady=10)
        
        self.imgPose = ImageTk.PhotoImage(self.img)
        width = self.imgPose.width()
        height = self.imgPose.height()
        if (self.imgPose.width() > 640): 
          width = 640
        if (self.imgPose.height() > 480):
          height = 480
        
        self.canvas = Canvas(self.frame, width = width, height = height, bd=0, xscrollcommand=self.xscrollbar.set, yscrollcommand=self.yscrollbar.set)
        self.canvas.grid(row=1, column=0, sticky=N+S+E+W)
        
        self.canvas.create_image(0,0,image=self.imgPose, anchor="nw")

        self.xscrollbar.config(command=self.canvas.xview)
        self.yscrollbar.config(command=self.canvas.yview)
        self.canvas.config(scrollregion=self.canvas.bbox(ALL))
        self.canvas.bind("<Button-1>",self.drawHomeRobotIcon)
        self.canvas.bind("<B3-Motion>",self.home_rotate)
        self.canvas.bind("r", self.home_rotate_with_key)
        self.riiCanvas = None 
        self.frame.pack()
                
    def quit(self):
      os.system("killall -9 xterm")
      self.parent.destroy()
      
    def kill_demo(self):
      os.system("killall -9 xterm")
      
    def setPorts(self):
      self.robotPort = self.robotPortText.get()
      self.laserPort = self.laserPortText.get()
      self.laserPort2 = self.laserPortText2.get()
      self.joyPort = self.joyPortText.get()
      
    def saveConfigFile(self):
      f = open('lastConfigUsed', 'w')
      f.write("[Config]\n")
      f.write("home_x: %s\n"%self.home_pos_x)
      f.write("home_y: %s\n"%self.home_pos_y)
      f.write("home_th: %s\n"%self.home_angle)
      f.write("init_x: %s\n"%self.robot_pos_x)
      f.write("init_y: %s\n"%self.robot_pos_y)
      f.write("init_a: %s\n"%self.robot_angle)
      f.write("robotport: %s\n"%self.robotPort)
      f.write("laserport: %s\n"%self.laserPort)
      f.write("laserport2: %s\n"%self.laserPort2)
      f.write("joyport: %s\n"%self.joyPort)
      f.write("kbenabled: %s\n"%self.kb.get())
      f.write("dotdetectorenabled: %s\n"%self.dd.get())
      f.write("combine_pnpenabled: %s\n"%self.combine_pnp.get())
      f.write("loggerenabled: %s\n"%self.logger.get())
      f.write("robotname: %s\n"%self.robot_ddm.get())

    def tryLoadOldConfig(self):
      try:
        self.oldConfigs = {}
        self.Config = ConfigParser.ConfigParser()
        self.Config.read("lastConfigUsed")
        for option in self.Config.options("Config"):
          self.oldConfigs[option] = self.Config.get("Config", option)
        print "Successfully loaded old configuration"
      
      except:
        print "Cound not load old config"
        
    def loadPorts(self):
      if "robotport" in self.oldConfigs:
        self.robotPort = self.oldConfigs["robotport"]
      else:
        self.robotPort = "/dev/ttyACM0"
      
      if "laserport" in self.oldConfigs:
        self.laserPort = self.oldConfigs["laserport"]
      else:
        self.laserPort = "/dev/ttyACM1"
      
      if "laserport2" in self.oldConfigs:
        self.laserPort2 = self.oldConfigs["laserport2"]
      else:
        self.laserPort2 = "/dev/ttyACM2"

      if "joyport" in self.oldConfigs:
        self.joyPort = self.oldConfigs["joyport"]
      else:
        self.joyPort = "/dev/ttyACM3"

def main():

    root = tk.Tk()
    DIP(root)
    root.geometry("650x320")
    root.mainloop()  


if __name__ == '__main__':
    main()
