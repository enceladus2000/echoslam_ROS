print(g())

def main():
	print('main func')
	print(g())

def g():
	a = f()
	return a*2

def f():
	return 69

if __name__ == '__main__':
	main()
