import io


# Training Set
pos_training = io.open("/var/lib/mysql-files/TRAIN_POS_G_trust2012_4@hotmail.it.csv","r",encoding='utf-8').read()
neg_training = io.open("/var/lib/mysql-files/TRAIN_POS_B_trust2012_4@hotmail.it.csv","r",encoding='utf-8').read()


# Test Set
pos_test = io.open("/var/lib/mysql-files/TEST_POS_trust2012@hotmail.it_all.csv","r",encoding='utf-8').read()
neg_test = io.open("/var/lib/mysql-files/TEST_NEG_trust2012@hotmail.it_all.csv","r",encoding='utf-8').read()
