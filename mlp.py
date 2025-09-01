from sklearn.neural_network import MLPRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import r2_score
import pandas as pd
import joblib  # αν παλιά έκδοση sklearn

dataset=pd.read_csv('GFG.csv')
X=dataset.iloc[:,0:6]
Y=dataset.iloc[:,6]
X_train,X_test,y_train,y_test=train_test_split(X,Y,test_size=0.20,random_state=None)

model=MLPRegressor(hidden_layer_sizes=(220,500,10),max_iter=2400,activation='relu',solver='lbfgs',learning_rate='adaptive',learning_rate_init=0.002)
model.fit(X=X_train,y=y_train)
y_pred=model.predict(X_test)

r2 = r2_score(y_test, y_pred)
print(r2)
joblib.dump(model, 'mlp_model.pkl')