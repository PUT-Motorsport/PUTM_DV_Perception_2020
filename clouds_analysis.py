from numpy.lib.function_base import average
import pandas as pd
import numpy as np
from pandas.core.frame import DataFrame
from pandas.io.feather_format import read_feather
import pickle
import matplotlib.pyplot as plt
from pandasgui import show
from sklearn.linear_model import LogisticRegression
from sklearn.tree import DecisionTreeClassifier
from sklearn.svm import SVC
from sklearn.ensemble import RandomForestClassifier
from sklearn.neural_network import MLPClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error, accuracy_score, precision_score, recall_score
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import plot_confusion_matrix


# with open('cones.pkl', "rb") as fh:
#   data = pickle.load(fh)
#
# df = data.copy()
#
# # if color indexes saved as strings
# df['color'] = pd.to_numeric(df['color'], errors='coerce')
#
# # remove NaN value from colors
# df['color'].replace("", float("NaN"), inplace=True)
# df.dropna(subset = ["color"], inplace=True)
#
# # convert color to int
# df['color'] = df['color'].astype(int)

# df.to_parquet('cones.prqt')


df = pd.read_parquet('cones.prqt')
colors = [None, 'yellow', 'blue', 'orange']

columns = ['color', 'mean', 'median', 'std', 'min', 'max', '25%', '50%', '75%']


def get_row(cone):
    row = {}
    row[columns[0]] = colors[cone['color']]
    row[columns[1]] = np.mean(cone['intensity'])
    row[columns[2]] = np.median(cone['intensity'])
    row[columns[3]] = np.std(cone['intensity'])
    row[columns[4]] = np.min(cone['intensity'])
    row[columns[5]] = np.max(cone['intensity'])
    row[columns[6]] = np.quantile(cone['intensity'], 0.25)
    row[columns[7]] = np.quantile(cone['intensity'], 0.5)
    row[columns[8]] = np.quantile(cone['intensity'], 0.75)
    return row


def show_cone_clouds(df):
    for cone in df.iterrows():
        plt.scatter(cone[1]['x'], cone[1]['z'], c=cone[1]['intensity'], cmap='rainbow')
        plt.ylim(-0.35, 0.3)
        plt.title(colors[cone[1]['color']])
        plt.show()


def features_train_attempt():
    features_df = pd.DataFrame(columns=columns)

    for cone in df.iterrows():
          cone = cone[1]
          row = get_row(cone)
          features_df = features_df.append(row, ignore_index=True)

    features_df.boxplot(column=columns[1:], by='color')
    plt.show()

    le = LabelEncoder()
    le.fit(features_df.iloc[:, 0])
    target = le.transform(features_df.iloc[:, 0])

    X_train, X_test, y_train, y_test = train_test_split(features_df.iloc[:, 1:6], target, test_size=0.2, random_state=42,
                                                        stratify=target)

    lr = LogisticRegression(solver='lbfgs', max_iter=1000)
    dtc = DecisionTreeClassifier()
    svc = SVC()
    forest = RandomForestClassifier(n_estimators=50)
    mlp = MLPClassifier(max_iter=500, batch_size=64)

    lr.fit(X_train, y_train)
    dtc.fit(X_train, y_train)
    svc.fit(X_train, y_train)
    forest.fit(X_train, y_train)
    mlp.fit(X_train, y_train)

    preds_lr = lr.predict(X_test)
    preds_dtc = dtc.predict(X_test)
    preds_svc = svc.predict(X_test)
    preds_forest = forest.predict(X_test)
    preds_mlp = mlp.predict(X_test)

    mse_lr = mean_squared_error(y_test, preds_lr)
    recall_lr = recall_score(y_test, preds_lr, average='micro')
    acc_lr = accuracy_score(y_test, preds_lr)
    prec_lr = precision_score(y_test, preds_lr, average='micro')

    mse_dtc = mean_squared_error(y_test, preds_dtc)
    recall_dtc = recall_score(y_test, preds_dtc, average='micro')
    acc_dtc = accuracy_score(y_test, preds_dtc)
    prec_dtc = precision_score(y_test, preds_dtc, average='micro')

    mse_svc = mean_squared_error(y_test, preds_svc)
    recall_svc = recall_score(y_test, preds_svc, average='micro')
    acc_svc = accuracy_score(y_test, preds_svc)
    prec_svc = precision_score(y_test, preds_svc, average='micro')

    mse_forest = mean_squared_error(y_test, preds_forest)
    recall_forest = recall_score(y_test, preds_forest, average='micro')
    acc_forest = accuracy_score(y_test, preds_forest)
    prec_forest = precision_score(y_test, preds_forest, average='micro')

    mse_mlp = mean_squared_error(y_test, preds_mlp)
    recall_mlp = recall_score(y_test, preds_mlp, average='micro')
    acc_mlp = accuracy_score(y_test, preds_mlp)
    prec_mlp = precision_score(y_test, preds_mlp, average='micro')

    print(f'\n/* Logistic regression *\:\nMSE: {mse_lr}, Accuracy: {acc_lr}, Precision: {prec_lr}, Recall: {recall_lr}')
    print(f'\n/* SVM classifier *\:\nMSE: {mse_svc}, Accuracy: {acc_svc}, Precision: {prec_svc}, Recall: {recall_svc}')
    print(f'\n/* Decision tree *\:\nMSE: {mse_dtc}, Accuracy: {acc_dtc}, Precision: {prec_dtc}, Recall: {recall_dtc}')
    print(f'\n/* Random forest *\:\nMSE: {mse_forest}, Accuracy: {acc_forest}, Precision: {prec_forest}, Recall: {recall_forest}')
    print(f'\n/* MLP *\:\nMSE: {mse_mlp}, Accuracy: {acc_mlp}, Precision: {prec_mlp}, Recall: {recall_mlp}')

    plot_confusion_matrix(estimator=forest, X=X_test, y_true=y_test)
    inv_trans = le.inverse_transform([0, 1, 2])
    plt.title(f'0: {inv_trans[0]}, 1: {inv_trans[1]}, 2: {inv_trans[2]}')
    plt.show()

    # show how forest did
    for cone in df.iterrows():
        plt.scatter(cone[1]['x'], cone[1]['z'], c=cone[1]['intensity'], cmap='rainbow')
        plt.ylim(-0.35, 0.3)
        cone = cone[1]
        row = get_row(cone)
        data = pd.DataFrame(row, index=[0])
        pred = forest.predict(data.iloc[:, 1:6])
        plt.title(f"True: {row['color']}\n Pred: {le.inverse_transform(pred)[0]}")
        plt.show()


if __name__ == "__main__":
    features_train_attempt()
