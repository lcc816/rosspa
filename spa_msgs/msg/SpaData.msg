uint16 dialogId         # DialogId identifier set by the requester
uint16 payloadLength    # Length of the payload data
uint16 sequenceIndex    # The 1-indexed number of index of the message 
                        # within the sequence. the 'i' in 'i' of 'j'
uint16 sequenceCount    # The total number of messages within the sequence, 
                        # the 'j' in 'i' of 'j'
uint8 interfaceId       # xTEDS interface id
uint8 messageId         # xTEDS message Id 
uint8[] payload         # Message data payload