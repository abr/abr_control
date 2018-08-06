from abr_control.utils import DataHandler
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

dat = DataHandler(use_cache=True, db_name='dewolf2018neuromorphic')
tests = [
         'pd_no_weight_11',
         'pd_no_weight_10',
         'pd_no_weight_9',
        ]
n_runs = 9

def load_error(test, n_runs=9):
    error = []
    for run in range(0, n_runs):
        data = dat.load(params = ['error'],
                save_location='1lb_random_target/%s/session000/run%03d'%(test, run))
        kp = dat.load(params=['interface_kp'],
            save_location='1lb_random_target/%s/parameters/test_parameters'%test)['interface_kp']
        error.append(data['error'])
    return [error, kp]

plt.figure(figsize=(10,10))
for ii in range(0, n_runs):
    plt.subplot(3,3,ii+1)
    plt.ylabel('Error [m]')
    for test in tests:
        [err, kp] = load_error(test=test, n_runs=n_runs)
        if ii==0:
            plt.plot(err[ii], label='kp:\n %s'%kp)
        else:
            plt.plot(err[ii], label=test)
    if ii == 0:
        plt.legend(bbox_to_anchor=[-0.55, 0.5], loc='center left')
    else:
        plt.legend()
#plt.savefig('interface_kp_error_comparison.pdf')
plt.show()
