	CC = g++
FLAGS = -g -fpic 

SRCDIR=./src/
OBJDIR=./bin/
LIBDIR=./lib/
DIRS = $(SRCDIR) $(OBJDIR) $(LIBDIR) 

OBJ = Parser.o ParseBlock.o ParseProperty.o Checker.o StringUtil.o

POBJ = $(addprefix $(OBJDIR), $(OBJ))


all: example

example: $(DIRS)  libsparser.a libsparser.so $(OBJDIR)example

$(OBJDIR)example: $(SRCDIR)main.cpp 
	g++ -o $@ $< -L./lib -lsparser -lboost_regex-mt
	cp $(SRCDIR)polo $(OBJDIR)polo

libsparser.a : $(POBJ)
	ar rcs $(LIBDIR)$@ $(POBJ)

libsparser.so : $(POBJ)
	$(CC) -shared -o $(LIBDIR)$@ $(POBJ)


$(POBJ): $(OBJDIR)%.o : $(SRCDIR)%.cpp $(SRCDIR)%.h
	$(CC) -c $(FLAGS) $< -o $@

$(OBJDIR)Parser.o: $(SRCDIR)Parser.cpp $(SRCDIR)Parser.h $(SRCDIR)ParseContext.h
	g++ -c $(FLAGS) $< -o $@

$(OBJDIR)Checker.o: $(SRCDIR)Checker.cpp $(SRCDIR)Checker.h $(SRCDIR)ParseBlock.h
	g++ -c $(FLAGS) $< -o $@

$(OBJDIR)ParseBlock.o: $(SRCDIR)ParseBlock.cpp $(SRCDIR)ParseBlock.h $(SRCDIR)ParseContext.h $(SRCDIR)ParseProperty.h
	g++ -c $(FLAGS) $< -o $@



$(DIRS) : % :
	mkdir $@

clean:
	rm -f $(OBJDIR)*.o
	rm -f $(OBJDIR)example
	rm -f $(LIBDIR)libsparser.a
	rm -f $(LIBDIR)libsparser.so

rebuild: clean all

install: 
	@if [ ! -d /usr/local/include/sparser ]; then \
		mkdir /usr/local/include/sparser; \
	fi; \
	cp $(SRCDIR)/*.h /usr/local/include/sparser; \
	cp lib/libsparser.so /usr/local/lib; \
	cp lib/libsparser.a /usr/local/lib; 

uninstall: 
	rm -R /usr/local/include/sparser
	rm /usr/local/lib/libsparser.so
	rm /usr/local/lib/libsparser.a
	

