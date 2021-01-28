from spectroscopy import *

from sklearn import model_selection
from sklearn import metrics
from sklearn.cross_decomposition import PLSRegression
from sklearn.pipeline import Pipeline
from sklearn.linear_model import LinearRegression, Ridge
from sklearn.decomposition import PCA

import time
import pickle

import warnings
warnings.filterwarnings('ignore')


######################################################################
######################################################################
######################################################################
def benchmark_algorithms(alg, N_runs=1, N_other_all=[1, 5, 9], N_train_array=[6, 10, 15, 20], mix_measure_noise=3, if_background_Remove=False, if_verbose=False, if_plot_scatter=False):
    N_test, n_splits = 100, 3

    rmse_mean_all, rmse_std_all = [], []
    mae_mean_all, mae_std_all = [], []
    scoreFunCV = "neg_mean_absolute_error"

    file_name = alg + 'Noise' + str(mix_measure_noise) + 'Run' + str(N_runs)
    if if_background_Remove:
        file_name += 'BackgroundRemoved'

    if if_verbose:
        open(file_name + '.log', 'w')

    for N_other in N_other_all:
        mse_one_sweep = np.zeros((len(N_train_array), N_runs))
        mae_one_sweep = np.zeros((len(N_train_array), N_runs))

        t0 = time.time()
        for i, N_train in enumerate(N_train_array):
            for j in range(N_runs):
                X_train, y_train, X_test, y_test = create_simulation_fixed_other(N_other, mix_measure_noise, N_train, N_test, False)

                if if_background_Remove:
                    X_train, X_train_bkg = background_remove_full(X_train, 4, 'BS', 'Lieber')
                    X_test, X_test_bkg = background_remove_full(X_test, 4, 'BS', 'Lieber')

                CV = model_selection.KFold(n_splits=n_splits, shuffle=True)

                if alg is 'PLSR':
                    pipeline = Pipeline(steps=[('pls', PLSRegression())])
                    pipelineGridParam = dict(pls__n_components=np.arange(1, N_train * n_splits / (n_splits + 1), 1).astype('int'))
                if alg is 'PCR':
                    pipeline = Pipeline(steps=[('pca', PCA()), ('lr', LinearRegression())])
                    pipelineGridParam = dict(pca__n_components=np.arange(1, N_train * n_splits / (n_splits + 1), 1).astype('int'))
                if alg is 'RR':
                    pipeline = Pipeline(steps=[('rr', Ridge())])
                    pipelineGridParam = dict(rr__alpha=np.logspace(-20, 0, 100))

                estimator = model_selection.GridSearchCV(pipeline, cv=CV, iid=True, param_grid=pipelineGridParam, scoring=scoreFunCV, refit=True)

                estimator.fit(X_train, y_train)
                y_test_pred = estimator.best_estimator_.predict(X_test)

                mse_one_sweep[i, j] = metrics.mean_squared_error(y_test, y_test_pred)
                mae_one_sweep[i, j] = metrics.mean_absolute_error(y_test, y_test_pred)

                if if_plot_scatter:
                    fig, axc = plt.subplots()
                    axc.scatter(y_test, y_test_pred)

                if if_verbose:
                    print('******************************', file=open(file_name + '.log', 'a'))
                    print(pipelineGridParam, file=open(file_name + '.log', 'a'))
                    print(estimator.best_estimator_, file=open(file_name + '.log', 'a'))
                    print(np.sqrt(mse_one_sweep[i, j]), file=open(file_name + '.log', 'a'))
                    print(mae_one_sweep[i, j], file=open(file_name + '.log', 'a'))
                    print('******************************\n', file=open(file_name + '.log', 'a'))

        rmse_mean = np.mean(np.sqrt(mse_one_sweep), axis=1)
        rmse_std = np.std(np.sqrt(mse_one_sweep), axis=1)
        mae_mean = np.mean(mae_one_sweep, axis=1)
        mae_std = np.std(mae_one_sweep, axis=1)

        rmse_mean_all.append(rmse_mean)
        rmse_std_all.append(rmse_std)
        mae_mean_all.append(mae_mean)
        mae_std_all.append(mae_std)

        t1 = time.time()
        print(t1 - t0)

    pickle.dump((N_other_all, N_train_array, rmse_mean_all, rmse_std_all, mae_mean_all, mae_std_all), open(os.path.join('NewBenchmarks', file_name + '.p'), 'wb'))
    time.sleep(1)

    return file_name
