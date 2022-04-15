./compile.sh && clear && valgrind --log-file="ornis.log"  --track-origins=yes  --leak-check=full --show-leak-kinds=all ../../build/ornis/ornis
