import understandOrders2
# import pathscript
# import GeneralPurposeTest2 as GPT
# print U.sentences

# o=U.orderList()
# a=o.parseOrders(sentence=U.sentences)

def parseSentence(sentence):
    o = understandOrders2.orderList()
    a = o.parseOrders(sentence)
    return a
