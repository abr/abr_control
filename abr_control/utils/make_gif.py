import subprocess
def create(fig_loc, save_loc, save_name, delay=5):
    bashCommand = ("convert -delay %i -loop 0 -deconstruct -quantize"%delay
                   + " transparent -layers optimize -resize 1200x2000"
                   + " %s/*.png %s/%s.gif"
                   %(fig_loc, save_loc, save_name))
    print('Creating gif...')
    process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()
    print('Finished')
