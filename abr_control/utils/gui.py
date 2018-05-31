# Add link to sentdex youtube
#https://www.youtube.com/watch?v=A0gaXfM1UN0&index=2&list=PLQVvvaa0QuDclKx-QpC9wntnURXVJqLyk
import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation
from matplotlib import style
from PIL import Image, ImageTk

from abr_control.utils import DataHandler

dat = DataHandler(use_cache=True)

LARGE_FONT = ("Verdana", 12)
style.use("ggplot")

f = Figure(figsize=(5,5), dpi=100)
a = f.add_subplot(111)

def animate(i):
    pullData = open("SampleData.txt", "r").read()
    dataList = pullData.split('\n')
    xList = []
    yList = []
    for eachLine in dataList:
        if len(eachLine) > 1:
            x, y = eachLine.split(',')
            xList.append(int(x))
            yList.append(int(y))
    a.clear()
    a.plot(xList, yList)

class RunData(tk.Tk):

    def __init__(self, *args, **kwargs):

        tk.Tk.__init__(self, *args, **kwargs)
        tk.Tk.wm_title(self, 'abr_control data')

        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}

        for F in (StartPage, SearchPage, PlotPage):
            frame = F(container, self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")
        self.show_frame(StartPage)

    def show_frame(self, cont):
        frame = self.frames[cont]
        frame.tkraise()

class SearchPage(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Search", font=LARGE_FONT)
        label.pack(pady=10, padx=10)

        button1 = ttk.Button(self, text="Home",
                command=lambda: controller.show_frame(StartPage))
        button1.pack()
        button3 = ttk.Button(self, text="Plot",
                command=lambda: controller.show_frame(PlotPage))
        button3.pack()
        button2 = ttk.Button(self, text="Quit",
                command=self.quit)
        button2.pack()

        self.search_var = tk.StringVar()
        self.search_var.trace("w", self.update_list)
        self.entry = tk.Entry(self, textvariable=self.search_var, width=13)
        self.lbox = tk.Listbox(self, width=45, height=15)

    #     self.entry.grid(row=0, column=0, padx=10, pady=3)
        self.entry.pack()
    #     self.lbox.grid(row=1, column=0, padx=10, pady=3)
        self.lbox.pack()

        # Function for updating the list/doing the search.
        # It needs to be called here to populate the listbox.
        self.update_list()

        # canvas = tk.Canvas(self, width=200, height=200)
        # canvas.pack()
        img = Image.open('test_img.jpg')
        img = img.resize((200,200), Image.ANTIALIAS)
        img = ImageTk.PhotoImage(img)
        #canvas.create_image(20,20, image=img)
        label = tk.Label(image=img, width=200, height=200)
        label.image = img
        label.pack()

    def update_list(self, *args):
        search_term = self.search_var.get()

        # Just a generic list to populate the listbox
        # lbox_list = ['Adam', 'Lucy', 'Barry', 'Bob',
        #              'James', 'Frank', 'Susan', 'Amanda', 'Christie']
        lbox_list = dat.get_keys('testing/joint_space_training/')

        self.lbox.delete(0, tk.END)

        for item in lbox_list:
                if search_term.lower() in item.lower():
                    self.lbox.insert(tk.END, item)

def button_func(param):
    print(param)

class StartPage(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Home", font=LARGE_FONT)
        label.pack(pady=10, padx=10)

        # button to activate a function
        # button1 = tk.Button(self, text="Visit Page 1",
        #         command=lambda:button_func("print something"))
        # button1.pack()
        button2 = ttk.Button(self, text="Search",
                command=lambda: controller.show_frame(SearchPage))
        button2.pack()
        button3 = ttk.Button(self, text="Plot",
                command=lambda: controller.show_frame(PlotPage))
        button3.pack()
        button1 = ttk.Button(self, text="Quit",
                command=self.quit)
        button1.pack()

class PlotPage(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Plot", font=LARGE_FONT)
        label.pack(pady=10, padx=10)

        button1 = ttk.Button(self, text="Home",
                command=lambda: controller.show_frame(StartPage))
        button1.pack()
        button3 = ttk.Button(self, text="Search",
                command=lambda: controller.show_frame(SearchPage))
        button3.pack()
        button2 = ttk.Button(self, text="Quit",
                command=self.quit)
        button2.pack()

        canvas = FigureCanvasTkAgg(f, self)
        canvas.show()
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # show the matplotlib toolbar
        toolbar = NavigationToolbar2TkAgg(canvas, self)
        toolbar.update()
        canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

app = RunData()
ani = animation.FuncAnimation(f, animate, interval=1000)
app.mainloop()
