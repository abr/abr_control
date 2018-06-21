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
from abr_control.utils import DataHandler, PlotError

# import data handler
dat = DataHandler(use_cache=True)
pltE = PlotError()

# set some constants
LARGE_FONT = ("Verdana", 20)
MED_FONT = ("Verdana", 10)
SMALL_FONT = ("Verdana", 8)
style.use("ggplot")
global text_color
#text_color = 'white'
# charcoal grey
text_color = '#36454f'
global background_color
# charcoal grey
# background_color = '#36454f'
background_color = 'white'
global button_color
# abr blue
button_color = '#375580'
global button_text_color
button_text_color = '#d3d3d3'#'white'

f = Figure(figsize=(5,5), dpi=100)
a = f.add_subplot(111)

# global variable for searching in db
loc = ['/']
# list of tests to display data from in plot
disp_loc = []
# sets whether to display data passed the group 'session'
browse_datasets = False
# list of possible plotting variables
plotting_variables = ['q', 'dq', 'error', 'u', 'adapt', 'mean & ci']
global plotting_colors
plotting_colors = []
# boolean that triggers when a test is added or removed from the plotting list
update_plot = False
# list of selected variables to plot
var_to_plot = 'error'
last_plotted = var_to_plot
# variable for toggling whether to save current figure
save_figure = False

def live_plot(i):
    """
    The function that plots the selected tests and data
    """
    global save_figure
    global plotting_colors
    global update_plot
    global disp_loc
    global last_plotted

    if last_plotted != var_to_plot:
        update_plot = True

    if update_plot:
        #TODO update only when there is a change instead of periodically
        #TODO automatically update range if it is larger than the current max
        # clear our data before plotting
        a.clear()
        last_plotted = var_to_plot
        x_min = 0.0
        x_max = 0.1
        y_min = 0.0
        y_max = 0.1
        # cycle through selected tests to plot
        for count, test in enumerate(disp_loc):
            print('count:',count)
            print('len colors:',len(plotting_colors))
            if count+1 > len(plotting_colors):
                plotting_colors.append(np.around(np.random.rand(3,1), decimals=1))
            print('plotting %s' %test)
            legend_name = test.split('/')[-2]
            legend_name += ' %s'%var_to_plot
            print('PLOTTING COLORS: ', plotting_colors)

            # if we're interested in plotting error, we will average the error over
            # a run and plot the average of each run
            if var_to_plot == 'error':
                # set location and legend label
                location = '%ssession000/'%test
                # get a list of keys in the current group
                keys = dat.get_keys(location)
                print('plotting error')
                error = []
                for entry in keys:
                    # only look through the keys that point to a run group
                    if 'run' in entry:
                        # append the average error of every run to plot the error
                        # change over runs
                        d = dat.load(params=[var_to_plot],
                                save_location='%s%s'%(location,
                                entry))
                        d = np.mean(d[var_to_plot])
                        error.append(np.copy(d))
                # set our plotting range
                if max(error) > y_max:
                    y_max = max(error)
                if min(error) < y_min:
                    y_min = min(error)
                if len(error) > x_max:
                    x_max = len(error)
                a.set_xlim(x_min, x_max)
                a.set_ylim(y_min, y_max)
                a.plot(error, label=legend_name)

            elif var_to_plot == 'mean & ci':
                location = '%sproc_data/position'%test
                try:

                    d = dat.load(params=['mean', 'upper_bound', 'lower_bound'],
                            save_location=location)
                    print('plotting colors pre: ', plotting_colors[count])
                    pltE.plot_data(data=[d], show_plot=False, fig_obj=a,
                            colors=[plotting_colors[count]])
                except:
                    print('%s does not contain processed data'%location)
                    disp_loc.remove(test)

            # if we're not plotting error, then every other dataset will have
            # dim=N_JOINTS so we will plot the data from the final run of the
            # session
            else:
                # set location and legend label
                location = '%ssession000/'%test
                # get a list of keys in the current group
                keys = dat.get_keys(location)
                print('plotting %s' %var_to_plot)
                # get the highest numbered run group
                max_key = max([key for key in keys if 'run' in key])
                d = dat.load(params=[var_to_plot],
                        save_location='%s%s'%(location, max_key))
                # load data from highest numbered run
                try:
                    d = np.array(d[var_to_plot])
                    # set our plotting limits
                    if np.max(d) > y_max:
                        y_max = np.max(d)
                    if np.min(d) < y_min:
                        y_min = np.min(d)
                    if len(d) > x_max:
                        x_max = len(d)
                    a.set_xlim(x_min, x_max)
                    a.set_ylim(y_min, y_max)
                    a.plot(d, label=legend_name)
                except TypeError:
                    print('WARNING: %s%s does not contain the key %s'%(location, max_key,
                      var_to_plot))



        f.tight_layout()
        if var_to_plot != 'mean & ci':
            a.legend(loc=2)#bbox_to_anchor=(1.05,1), loc=2, borderaxespad=0.)
        if save_figure:
            a.figure.savefig('default_figure.pdf')
            save_figure = False
        update_plot = False

def save_figure_toggle(self):
    """
    a toggle that is changed based on a button click. it is momentarily
    toggled to save the current figure
    """
    global save_figure
    save_figure = True
    print('Figure Saved')

def clear_plot(self):
    """
    a toggle that is changed based on a button click and is used to clear the
    plot
    """
    global disp_loc
    global update_plot
    disp_loc = []
    update_plot = True

def popupmsg(msg):
    """
    generic function to pass in a string to appear in a popup message
    """
    popup = tk.Tk()
    popup.wm_title("!")
    label = tk.Label(popup, text=msg, font=MED_FONT)
    label.pack(side="top", fill="x", pady=10)
    B1 = ttk.Button(popup, text='OK', command=popup.destroy)
    B1.pack()
    popup.mainloop()

def go_back_loc_level(self):
    """
    function used to go back a search level, can be thought of as going back
    a directory
    """
    global loc
    loc = loc[:-1]
    #self.entry.delete(0, 'end')
    self.update_list()

def go_to_root_level(self):
    """
    Function used to reset database location to root location
    """
    global loc
    loc = ['/']
    self.update_list()

def toggle_browse_datasets(self):
    """
    Toggles the browse_datasets variable.

    If browse datasets is set to False
    we will stop going down the chain in the database once we reach a group
    that contains a 'session' group. At this point we want to save the
    selected test and plot it.
    If browse datasets is set to True then we will go further down the chain,
    passed the session group level and allow the user to view the save
    structure.
    """
    global browse_datasets
    browse_datasets = not browse_datasets

def add_img(self, size=[200,200], file_loc='test_img.jpg', row=0, column=0, *args):
    """
    A generic function for loading a provided photo into a specific row and
    column
    """
    img = Image.open(file_loc)
    img = img.resize((size[0], size[1]), Image.ANTIALIAS)
    img = ImageTk.PhotoImage(img)
    label = tk.Label(self, image=img, width=size[0], height=size[1])
    label.image = img
    label.grid(row=row, column=column, padx=10, pady=10)

class Page(tk.Tk):

    def __init__(self, *args, **kwargs):

        # instantiate our container
        tk.Tk.__init__(self, *args, **kwargs)
        tk.Tk.wm_title(self, 'abr_control data')

        container = tk.Frame(self)
        #container.configure(background='white')
        container.grid(row=0, column=0, sticky='nsew')
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

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

        # Example of another menubar entry that is not currently used
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

        # place our other pages into the master page
        for F in (StartPage, SearchPage):
            frame = F(container, self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")
        self.show_frame(StartPage)

    def show_frame(self, cont):
        """
        Function for pulling the selected frame to the front
        """
        frame = self.frames[cont]
        frame.tkraise()

class StartPage(tk.Frame):
    """
    The starting page when the gui is loaded
    """

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.configure(background=background_color)

        page_title = tk.Label(self, text='Home', font=LARGE_FONT)
        page_title.grid(row=0, column=1, padx=10)
        page_title.configure(background=background_color, foreground=text_color)

        main_text = tk.Label(self, text=('Welcome to the abr_control plotting GUI!'
            +'\n\nBrowse through your recorded tests in the \'Search\' Page.'
            + '\nAs you select tests from the list they will appear on the plot.'
            + '\nSelect them again to remove them, or click the \'Clear Plot\''
            + '\nbutton to clear the plot entirely.'
            + '\n\nThe default plotting parameter is End-Effector Distance to '
            + 'Target,\nbut this'
            + ' can be changed through the radio butons along the side of the'
            + ' plot.'), font=MED_FONT)
        main_text.grid(row=1, column=1)
        main_text.configure(background=background_color, foreground=text_color)

        # Add a button to take bring the search page to the front
        search_button = tk.Button(self, text="Search",
                command=lambda: controller.show_frame(SearchPage),
                bg=button_color, fg=button_text_color)
        search_button.grid(row=2,column=1)

        # Add a picture of the abr logo for funzies
        add_img(self, file_loc='abr_logo.jpg', row=0, column=0)

class SearchPage(tk.Frame):

    def __init__(self, parent, controller):
        # instantiate our frame
        tk.Frame.__init__(self, parent)
        self.configure(background=background_color)

        # create a left and right frame to simplify organization of grid
        frame_left = tk.Frame(self,parent)
        frame_left.grid(row=0,column=0, padx=10)
        frame_left.configure(background=background_color)

        frame_right = tk.Frame(self,parent)
        frame_right.grid(row=0,column=2, padx=10)
        frame_right.configure(background=background_color)

        page_title = tk.Label(frame_left, text="Search", font=LARGE_FONT)
        page_title.grid(row=0, column=1)
        page_title.configure(background=background_color, foreground=text_color)

        self.current_location_display = tk.StringVar()
        self.current_location_display.set(''.join(loc))

        # text printout to show our current search location
        current_location_label = tk.Label(frame_left,
                textvariable=self.current_location_display, font=MED_FONT)
        current_location_label.grid(row=2, column=0, columnspan=3)
        current_location_label.configure(background=background_color,
                foreground=text_color)

        # create our buttons
        home_button = tk.Button(frame_left, text="Home",
                command=lambda: go_to_root_level(self))
                #command=lambda: controller.show_frame(StartPage))
        home_button.grid(row=1, column=1)#, sticky='nsew')
        home_button.configure(background=button_color, foreground=button_text_color)

        back_button = tk.Button(frame_left, text="Back",
                command=lambda: go_back_loc_level(self))
        back_button.grid(row=1, column=0)#, sticky='nsew')
        back_button.configure(foreground=button_text_color,
                background=button_color)

        clear_plot_button = tk.Button(frame_left, text="Clear Plot",
                command=lambda: clear_plot(self))
        clear_plot_button.grid(row=1, column=2)#, sticky='nsew')
        clear_plot_button.configure(background=button_color,
                foreground=button_text_color)

        browse_datasets_button = tk.Button(frame_left, text="Browse Datasets",
                command=lambda: toggle_browse_datasets(self))
        browse_datasets_button.grid(row=5, column=1)#, sticky='nsew')
        browse_datasets_button.configure(foreground=button_text_color,
                background=button_color)

        # create our search bar and list box
        self.search_var = tk.StringVar()
        self.search_var.trace("w", self.update_list)
        self.entry = tk.Entry(frame_left, textvariable=self.search_var, width=13)
        self.lbox = tk.Listbox(frame_left, width=45, height=15, selectmode='MULTIPLE')
        self.lbox.bind('<<ListboxSelect>>', self.get_selection)

        self.entry.grid(row=3, column=0, columnspan=3, sticky='ew')#, sticky='nsew', columnspan=3)
        self.lbox.grid(row=4, column=0, columnspan=3)#, sticky='nsew', columnspan=3)
        self.entry.configure(background=background_color,
                foreground=text_color)
        self.lbox.configure(background=background_color, foreground=text_color)

        # Function for updating the list/doing the search.
        # It needs to be called here to populate the listbox.
        self.update_list()
        #values = [self.lbox.get(idx) for idx in self.lbox.curselection()]

        # Plotting Window
        canvas = FigureCanvasTkAgg(f, self)
        canvas.show()
        canvas.get_tk_widget().grid(row=0, column=1)#, ipadx=100, sticky='w')
        canvas.get_tk_widget().configure(background=background_color)

        # show the matplotlib toolbar
        #toolbar = NavigationToolbar2TkAgg(canvas, self)
        # toolbar.update()
        # canvas._tkcanvas.grid(row=0, column=5, columnspan=10)

	# initialize radio buttons
        self.var_to_plot = tk.StringVar()
        self.var_to_plot.set(var_to_plot)
        for ii, var in enumerate(plotting_variables):
            var_to_plot_radio = tk.Radiobutton(frame_right, text=var,
                    variable=self.var_to_plot,
                    value=var, command=lambda: self.update_var_to_plot(self))
            var_to_plot_radio.grid(row=ii, column=0, ipadx=20, sticky='ew')#, sticky='nsew')
            var_to_plot_radio.configure(background=button_color,
                    foreground=button_text_color)


    def update_var_to_plot(self, *args):
        """updates the global variable of what data to plot"""
        global var_to_plot
        var_to_plot = self.var_to_plot.get()

    def get_selection(self, *args):
        """
        get the selection from the listbox and update the list and search
        location accordingly based on button selection and location
        """
        global loc
        global disp_loc
        global plotting_colors
        global update_plot

        # delete the current search
        self.entry.delete(0, 'end')
        # get cursor selection and update db search location
        index = int(self.lbox.curselection()[0])
        value = self.lbox.get(index)
        print('You selected item %d: "%s"' % (index, value))
        # append the most recent selection to our search location
        loc.append('%s/'%value)

        # check if we're pointing at a dataset, if so go back one level
        print('FIRST ATTEMPT TO LOAD: ', ''.join(loc))
        keys = dat.get_keys(''.join(loc))
        # if keys are None, then we are pointing at a dataset
        if keys is None:
            # if the user selects the browse_datasets button, then they want to
            # view the save structure passed the session group level
            if browse_datasets:
                print('deep dive: loading ', (loc[-1])[:-1])
                print('looking in: ', ''.join(loc[:-1]))
                print('PARAMS: ', loc[-1][:-1])
                print('LOC: ', ''.join(loc[:-1]))
                print("CURRENT DISPLAY LIST: ", disp_loc)
                loaded_data = dat.load(params=[(loc[-1])[:-1]],
                        save_location=''.join(loc[:-1]))
                # print the selected dataset to the terminal
                print(loc[-2], '\n', loaded_data)

            # if browse_datasets is not selected then go back a level in the
            # search
            else:
                print('Error: ', dat.load(params='error', save_location=''.join(loc)))
                print('loc points to dataset')
            go_back_loc_level(self)


        # if keys do exist, check if we're at the session level, at which point
        # we should plot data, not go further down in the search, unless
        # browse_datasets is set to True
        elif not browse_datasets and any('session' in s for s in keys):
                # check if the selection is already in our list, if so remove it
                test_name = ''.join(loc)
                if test_name in disp_loc:
                    index = disp_loc.index(test_name)
                    disp_loc.remove(test_name)
                    # remove the entry of plotting colors that corresponds to
                    # the test being removed
                    del plotting_colors[index]
                else:
                    disp_loc.append(''.join(loc))
                print("CURRENT DISPLAY LIST: ", disp_loc)
                update_plot = True
                go_back_loc_level(self)

        # if the selection takes us to the next level of groups then erase the
        # search bar
        #else:
        self.update_list()


    def update_list(self, *args):
        """
        Function that updates the listbox based on the current search location
        """
        global loc
        self.current_location_display.set(''.join(loc))
        search_term = self.search_var.get()

        # pull keys from the database
        lbox_list = dat.get_keys(''.join(loc))

        self.lbox.delete(0, tk.END)

        for item in lbox_list:
            if search_term.lower() in item.lower():
                self.lbox.insert(tk.END, item)

app = Page()
#app.geometry("1280x720")
ani = animation.FuncAnimation(f, live_plot, interval=1000)
app.mainloop()
