from spectroscopy import *

from scipy.interpolate import InterpolatedUnivariateSpline
from scipy.stats import poisson

from sklearn import model_selection
from sklearn import ensemble
from sklearn import preprocessing
from sklearn import metrics

from sklearn.pipeline import Pipeline
from sklearn.cross_decomposition import PLSRegression
from sklearn.decomposition import PCA

import warnings
import time

np.set_printoptions(precision=5, suppress=True, linewidth=150)
plt.rcParams['mathtext.default'] = 'regular'


##############################################################################
##############################################################################
##############################################################################
def predictionAnalysisNoScale(X_all, y_all, group_all, if_double_CV, inner_CV, outer_CV, score_fun_CV='neg_mean_squared_error', group_test=[], if_verbose=True):
    pipeline = Pipeline(steps=[('pls', PLSRegression(scale=False))])
    pipeline_grid_param = dict(pls__n_components=np.arange(1, 30, 1).astype('int'))
    estimator = model_selection.GridSearchCV(pipeline, cv=inner_CV, iid=True, param_grid=pipeline_grid_param, scoring=score_fun_CV, refit=True)

    ######################################################################
    ######################################################################
    if if_double_CV:
        y_all_pred = np.empty_like(y_all)

        for train_index, test_index in outer_CV.split(X_all, y_all, group_all):
            X_train, X_test = X_all[train_index, :], X_all[test_index, :]
            y_train, y_test = y_all[train_index], y_all[test_index]

            estimator.fit(X_train, y_train)
            y_test_pred = estimator.best_estimator_.predict(X_test).flatten()

            if if_verbose:
                print(estimator.best_estimator_.steps[-1][-1])
                print('Current R2:', metrics.r2_score(y_test, y_test_pred))

            y_all_pred[test_index] = y_test_pred

        if if_verbose:
            print('\nOverall R2:', metrics.r2_score(y_all, y_all_pred))

        return y_all, y_all_pred, group_all

    else:
        test_index = np.array([np.where(group_all == idx)[0] for idx in group_test]).flatten()
        train_index = np.delete(np.arange(len(y_all)), test_index)

        X_train, X_test = X_all[train_index, :], X_all[test_index, :]
        y_train, y_test = y_all[train_index], y_all[test_index]

        estimator.fit(X_train, y_train)
        y_test_pred = estimator.best_estimator_.predict(X_test).flatten()

        if if_verbose:
            print(estimator.best_estimator_.steps[-1][-1])
            print('Test R2:', metrics.r2_score(y_test, y_test_pred))

        return y_test, y_test_pred, group_test


##############################################################################
##############################################################################
##############################################################################
def predictionAnalysis(X_all, y_all, group_all, if_double_CV, inner_CV, outer_CV, score_fun_CV='neg_mean_squared_error', group_test=[], if_verbose=True):
    pipeline = Pipeline(steps=[('scaler', preprocessing.StandardScaler()), ('pls', PLSRegression())])
    pipeline_grid_param = dict(pls__n_components=np.arange(1, 30, 1).astype('int'))
    estimator = model_selection.GridSearchCV(pipeline, cv=inner_CV, iid=True, param_grid=pipeline_grid_param, scoring=score_fun_CV, refit=True)

    ######################################################################
    ######################################################################
    if if_double_CV:
        y_all_pred = np.empty_like(y_all)

        for train_index, test_index in outer_CV.split(X_all, y_all, group_all):
            X_train, X_test = X_all[train_index, :], X_all[test_index, :]
            y_train, y_test = y_all[train_index], y_all[test_index]

            estimator.fit(X_train, y_train)
            y_test_pred = estimator.best_estimator_.predict(X_test).flatten()

            if if_verbose:
                print(estimator.best_estimator_.steps[-1][-1])
                print('Current R2:', metrics.r2_score(y_test, y_test_pred))

            y_all_pred[test_index] = y_test_pred

        if if_verbose:
            print('\nOverall R2:', metrics.r2_score(y_all, y_all_pred))

        return y_all, y_all_pred, group_all

    else:
        test_index = np.array([np.where(group_all == idx)[0] for idx in group_test]).flatten()
        train_index = np.delete(np.arange(len(y_all)), test_index)

        X_train, X_test = X_all[train_index, :], X_all[test_index, :]
        y_train, y_test = y_all[train_index], y_all[test_index]

        estimator.fit(X_train, y_train)
        y_test_pred = estimator.best_estimator_.predict(X_test).flatten()

        if if_verbose:
            print(estimator.best_estimator_.steps[-1][-1])
            print('Test R2:', metrics.r2_score(y_test, y_test_pred))

        return y_test, y_test_pred, group_test


##############################################################################
##############################################################################
##############################################################################
def predictionAnalysisNonlinear(X_all, y_all, group_all, if_double_CV, inner_CV, outer_CV, score_fun_CV='neg_mean_squared_error', group_test=[], if_verbose=True):
    # pipeline = Pipeline(steps=[('scaler', preprocessing.StandardScaler()), ('pca', PCA()), ('etr', ensemble.ExtraTreesRegressor(n_estimators=300, bootstrap=False, n_jobs=1))])
    # pipeline_grid_param = \
    #     {'pca__n_components': [5, 6, 7, 8, 9, 10, 11, 12],
    #      'etr__n_estimators': [100, 300],
    #      'etr__max_features': [0.5, 0.75, 1]}
    # estimator = model_selection.GridSearchCV(pipeline, cv=inner_CV, iid=True, param_grid=pipeline_grid_param, scoring=score_fun_CV, refit=True)

    pipeline = Pipeline(steps=[('scaler', preprocessing.StandardScaler()), ('etr', ensemble.ExtraTreesRegressor(n_estimators=300, bootstrap=False, n_jobs=4))])
    pipeline_grid_param = \
        {'etr__n_estimators': [100, 300, 1000, 3000, 5000],
         'etr__max_features': [0.5, 0.75, 1]}
    estimator = model_selection.GridSearchCV(pipeline, cv=inner_CV, iid=True, param_grid=pipeline_grid_param, scoring=score_fun_CV, refit=True)

    ######################################################################
    ######################################################################
    if if_double_CV:
        y_all_pred = np.empty_like(y_all)

        for train_index, test_index in outer_CV.split(X_all, y_all, group_all):
            X_train, X_test = X_all[train_index, :], X_all[test_index, :]
            y_train, y_test = y_all[train_index], y_all[test_index]

            estimator.fit(X_train, y_train)
            y_test_pred = estimator.best_estimator_.predict(X_test).flatten()

            if if_verbose:
                print(estimator.best_estimator_.steps[-1][-1])
                print('Current R2:', metrics.r2_score(y_test, y_test_pred))

            y_all_pred[test_index] = y_test_pred

        if if_verbose:
            print('\nOverall R2:', metrics.r2_score(y_all, y_all_pred))

        return y_all, y_all_pred, group_all

    else:
        test_index = np.array([np.where(group_all == idx)[0] for idx in group_test]).flatten()
        train_index = np.delete(np.arange(len(y_all)), test_index)

        X_train, X_test = X_all[train_index, :], X_all[test_index, :]
        y_train, y_test = y_all[train_index], y_all[test_index]

        estimator.fit(X_train, y_train)
        y_test_pred = estimator.best_estimator_.predict(X_test).flatten()

        if if_verbose:
            print(estimator.best_estimator_.steps[-1][-1])
            print('Test R2:', metrics.r2_score(y_test, y_test_pred))

        return y_test, y_test_pred, group_test