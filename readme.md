### How to run bot.py without main.py

main.py is the program that automatically sets up and runs all bots given a teamsize. Leave that for later.

Note: The bot_id's are zero indexed. The bot with id=0 is the first to initiate the transmission chain. Once it transmits, the others follow sequentially in a loop. So you have to first init the bots with non-zero ids first, _then_ run bot #0. See how below.

To start a team of say, 3 bots, first run the following command 2 times in 2 different terminals.

`rosrun echoslam bot.py --id=x --teamsize=3`

where x is {1, 2}

_Then_ run bot #0

`rosrun echoslam bot.py --id=0 --teamsize=3`

Thanks


