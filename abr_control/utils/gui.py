# Add link to sentdex youtube
#https://www.youtube.com/watch?v=A0gaXfM1UN0&index=2&list=PLQVvvaa0QuDclKx-QpC9wntnURXVJqLyk
#TODO Tutorial 19 adds help button option to walk through gui
"""
sudo apt-get build-dep python-imaging
sudo apt-get install libjpeg8 libjpeg62-dev libfreetype6 libfreetype6-dev
sudo pip install Pillow
"""
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

import numpy as np
from abr_control.utils import DataHandler

dat = DataHandler(use_cache=True)

LARGE_FONT = ("Verdana", 12)
MED_FONT = ("Verdana", 10)
SMALL_FONT = ("Verdana", 8)
style.use("ggplot")

f = Figure(figsize=(5,5), dpi=100)
a = f.add_subplot(111)

# global variable for searching in db
loc = ['/']
# list of tests to display data from
disp_loc = []
# sets whether to display data passed the group 'session'
restrict_display = True
# list of possible plotting variables
plotting_variables = ['q', 'dq', 'error', 'u', 'adapt']
# list of selected variables to plot
var_to_plot = 'error'
save_figure = False

def live_plot(i):
    global save_figure
    #TODO update only when there is a change instead of periodically
    # pullData = open("SampleData.txt", "r").read()
    # dataList = pullData.split('\n')
    # xList = []
    # yList = []
    # for eachLine in dataList:
    #     if len(eachLine) > 1:
    #         x, y = eachLine.split(',')
    #         xList.append(int(x))
    #         yList.append(int(y))
    a.clear()
    x_min = 0
    x_max = 1
    y_min = 0
    y_max = 1
    for test in disp_loc:
        print('var to plot: ', var_to_plot)
        location = '%ssession000/'%test
        legend_name = test.split('/')[-2]
        legend_name += ' %s'%var_to_plot
        keys = dat.get_keys(location)
        if var_to_plot == 'error':
            error = []
            for entry in keys:
                if 'run' in entry:
                    #print('entry: ', entry)
                    #if 'run' in entry:
                        # print(dat.load(params=['error'],
                        #         save_location='%s%s'%(location, entry)))
                    #print('entry: ', entry)
                    d = dat.load(params=[var_to_plot],
                            save_location='%s%s'%(location,
                            entry))
                    d = np.mean(d[var_to_plot])
                    error.append(np.copy(d))
            #print('error end: ', error)
            if max(error) > y_max:
                y_max = max(error)
            if min(error) < y_min:
                y_min = min(error)
            if len(error) > x_max:
                x_max = len(error)
            a.set_xlim(x_min, x_max)
            a.set_ylim(y_min, y_max)
            a.plot(error, label=legend_name)
        else:
            max_key = max([key for key in keys if 'run' in key])
            d = dat.load(params=[var_to_plot],
                    save_location='%s%s'%(location, max_key))
            d = np.array(d[var_to_plot])
            #print('key used: ', max_key)
            #print('loaded: ', d)
            if np.max(d) > y_max:
                y_max = np.max(d)
            if np.min(d) < y_min:
                y_min = np.min(d)
            if len(d) > x_max:
                x_max = len(d)
            a.set_xlim(x_min, x_max)
            a.set_ylim(y_min, y_max)
            a.plot(d, label=legend_name)
            

    a.legend(bbox_to_anchor=(0,1.02,1,.102), loc=1)
    if save_figure:
        a.figure.savefig('default_figure.pdf')
        save_figure = False

def save_figure_toggle(self):
    global save_figure
    save_figure = True
    print('Figure Saved')

def clear_plot(self):
    global disp_loc
    disp_loc = []

def popupmsg(msg):
    popup = tk.Tk()

    print(msg)
    popup.wm_title("!")
    print(msg)
    label = tk.Label(popup, text=msg, font=MED_FONT)
    print(msg)
    label.pack(side="top", fill="x", pady=10)
    print(msg)
    B1 = ttk.Button(popup, text='OK', command=popup.destroy)
    print(msg)
    B1.pack()
    print(msg)
    popup.mainloop()
    print(msg)

# def change_param(param):
#     global param_to_plot
#     param_to_plot = param
#
# def get_db_loc(loc):
#     global loc
#
# def update_plot_param_menu(self):
#     if loc == '/':
#         self.param_menu.delete()
#

def go_back_loc_level(self):
    global loc
    loc = loc[:-1]
    #self.entry.delete(0, 'end')
    self.update_list()

def toggle_restrict_display(self):
    global restrict_display
    restrict_display = not restrict_display

def add_img(self, size=[200,200], file_loc='test_img.jpg', row=0, column=0, *args):
    img = Image.open(file_loc)
    img = img.resize((size[0], size[1]), Image.ANTIALIAS)
    img = ImageTk.PhotoImage(img)
    label = tk.Label(self, image=img, width=size[0], height=size[1])
    label.image = img
    label.grid(row=row, column=column)

class Page(tk.Tk):

    def __init__(self, *args, **kwargs):

        tk.Tk.__init__(self, *args, **kwargs)
        tk.Tk.wm_title(self, 'abr_control data')

        container = tk.Frame(self)
        container.grid(row=0, column=0, sticky='nsew')
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)
        # container.pack(side="top", fill="both", expand=True)
        # container.grid_rowconfigure(0, weight=1)
        # container.grid_columnconfigure(0, weight=1)

        # menu bar at top
        menubar = tk.Menu(container)
        # define main file menu
        filemenu = tk.Menu(menubar, tearoff=0)
        filemenu.add_command(label="Save Figure",
                command=lambda: save_figure_toggle(self))
                #command=lambda:popupmsg(msg="Not Supported Yet"))
        filemenu.add_separator()
        filemenu.add_command(label="Exit", command=quit)
        # place file menu
        menubar.add_cascade(label="File", menu=filemenu)

        # # define parameter to plot menu
        # self.param_menu = tk.Menu(menubar, tearoff=1)
        # self.param_menu.add_command(label="None",
        #         command=lambda:popupmsg(
        #             msg=("There are no parameters to select from from the"
        #             + " current database group")))
        # self.param_menu.add_command(label="Error",
        #         command=lambda:changeParam("error"))
        # # place parameter menu bar
        # menubar.add_cascade(label="Plotting Parameters", menu=self.param_menu)

        tk.Tk.config(self, menu=menubar)

        self.frames = {}

        for F in (StartPage, SearchPage):
            frame = F(container, self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")
        self.show_frame(StartPage)

    def show_frame(self, cont):
        frame = self.frames[cont]
        frame.tkraise()

class StartPage(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        #label = tk.Label(self, text="Home", font=LARGE_FONT)
        label = tk.Label(self, text=('Welcome to the abr_control plotting GUI!'
            +'\n\nBrowse through your recorded tests in the \'Search\' Page. Any'
            + ' corresponding images that are saved to the test will be '
            + 'displayed'
            + '\n\nSwitch to the plotting screen to see a live plot of your'
            + ' selected tests'
            + '\n\nThe default plotting parameter is First Order Error, but this'
            + ' can be changed through the check boxes along the side of the'
            + ' \'Plotting\' page'), font=MED_FONT)
        label.grid(row=1, column=0)

        button2 = ttk.Button(self, text="Search",
                command=lambda: controller.show_frame(SearchPage))
        button2.grid(row=2,column=0)
        # button3 = ttk.Button(self, text="Plot",
        #         command=lambda: controller.show_frame(PlotPage))
        # button3.grid(row=3,column=0)

        add_img(self, file_loc='abr_logo.jpg', row=0, column=0)

class SearchPage(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        frame_left = tk.Frame(self,parent)
        frame_left.grid(row=0,column=0)
        frame_right = tk.Frame(self,parent)
        frame_right.grid(row=0,column=2)
        label = tk.Label(frame_left, text="Search", font=LARGE_FONT)
        label.grid(row=0, column=1)#, sticky='nsew')
        self.loc_label_var = tk.StringVar()
        self.loc_label_var.set(''.join(loc))

        loc_label = tk.Label(frame_left, textvariable=self.loc_label_var, font=MED_FONT)
                #command=lambda: update_loc_disp(self))
        loc_label.grid(row=2, column=0, columnspan=3)

        button1 = ttk.Button(frame_left, text="Home",
                command=lambda: controller.show_frame(StartPage))
        button1.grid(row=1, column=1)#, sticky='nsew')
        button2 = ttk.Button(frame_left, text="Back",
                command=lambda: go_back_loc_level(self))
        button2.grid(row=1, column=0)#, sticky='nsew')
        button3 = ttk.Button(frame_left, text="Clear Plot",
                command=lambda: clear_plot(self))
        button3.grid(row=1, column=2)#, sticky='nsew')

        button4 = ttk.Button(frame_left, text="Restrict List",
                command=lambda: toggle_restrict_display(self))
        button4.grid(row=4, column=1)#, sticky='nsew')

        self.search_var = tk.StringVar()
        self.search_var.trace("w", self.update_list)
        # self.selected_var = tk.StringVar()
        # self.selected_var.trace("w", self.get_selection)
        self.entry = tk.Entry(frame_left, textvariable=self.search_var, width=13)
        self.lbox = tk.Listbox(frame_left, width=45, height=15, selectmode='MULTIPLE')
        self.lbox.bind('<<ListboxSelect>>', self.get_selection)

        self.entry.grid(row=3, column=0, columnspan=3)#, sticky='nsew', columnspan=3)
        self.lbox.grid(row=4, column=0, columnspan=3)#, sticky='nsew', columnspan=3)

        # Function for updating the list/doing the search.
        # It needs to be called here to populate the listbox.
        self.update_list()
        values = [self.lbox.get(idx) for idx in self.lbox.curselection()]

        # Plotting Window
        canvas = FigureCanvasTkAgg(f, self)
        canvas.show()
        canvas.get_tk_widget().grid(row=0, column=1)#, rowspan=4, columnspan=4)

        # show the matplotlib toolbar
        #toolbar = NavigationToolbar2TkAgg(canvas, self)
        # toolbar.update()
        # canvas._tkcanvas.grid(row=0, column=5, columnspan=10)

	# initialize radio button
        self.rad1_var = tk.StringVar()
        self.rad1_var.set(var_to_plot)
        ii=0
        for var in plotting_variables:
            radio1 = ttk.Radiobutton(frame_right, text=var, variable=self.rad1_var,
                    value=var, command=lambda: self.update_var_to_plot(self))
            radio1.grid(row=ii, column=0, ipadx=20, sticky='w')#, sticky='nsew')
            ii += 1
	

    def update_var_to_plot(self, *args):
        global var_to_plot
        #if self.rad1_var in var_to_plot:
        #    var_to_plot.remove(self.rad1_var)
        #else:
        #    var_to_plot.append(rad1_var)

        #print("VARIABLES TO PLOT: ", var_to_plot)
        var_to_plot = self.rad1_var.get()

    def get_selection(self, *args):
        global loc
        global disp_loc

        # get cursor selection and update db search location
        index = int(self.lbox.curselection()[0])
        value = self.lbox.get(index)
        print('You selected item %d: "%s"' % (index, value))
        loc.append('%s/'%value)

        # check if we're pointing at a dataset, if so go back one level
        keys = dat.get_keys(''.join(loc))
        if keys is None:
            print('Error: ', dat.load(params='error', save_location=''.join(loc)))
            print('loc points to dataset')
            go_back_loc_level(self)

        # if keys do exist, check if we're at the session level, at which point
        # we should display data, not go further down in the search
        elif restrict_display:
            if any('session' in s for s in keys):
                # check if the selection is already in our list, if so remove it
                if ''.join(loc) in disp_loc:
                    disp_loc.remove(''.join(loc))
                else:
                    disp_loc.append(''.join(loc))
                print("CURRENT DISPLAY LIST: ", disp_loc)
                go_back_loc_level(self)

        # if the selection takes us to the next level of groups then erase the
        # search bar
        else:
            self.entry.delete(0, 'end')
        self.loc_label_var.set(''.join(loc))
        self.update_list()

    def update_list(self, *args):
        global loc
        search_term = self.search_var.get()

        # pull keys from the database
        lbox_list = dat.get_keys(''.join(loc))

        self.lbox.delete(0, tk.END)

        for item in lbox_list:
            if search_term.lower() in item.lower():
                self.lbox.insert(tk.END, item)

# class PlotPage(tk.Frame):
#
#     def __init__(self, parent, controller):
#         tk.Frame.__init__(self, parent)
#         label = tk.Label(self, text="Plot", font=LARGE_FONT)
#         label.pack(pady=10, padx=10)
#
#         button1 = ttk.Button(self, text="Home",
#                 command=lambda: controller.show_frame(StartPage))
#         button1.pack()
#         button3 = ttk.Button(self, text="Search",
#                 command=lambda: controller.show_frame(SearchPage))
#         button3.pack()

        # canvas = FigureCanvasTkAgg(f, self)
        # canvas.show()
        # canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        #
        # # show the matplotlib toolbar
        # toolbar = NavigationToolbar2TkAgg(canvas, self)
        # toolbar.update()
        # canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

app = Page()
#app.geometry("1280x720")
ani = animation.FuncAnimation(f, live_plot, interval=1000)
app.mainloop()
