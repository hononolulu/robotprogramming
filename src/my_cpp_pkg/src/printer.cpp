#include <iostream>

int main(int argc, char *argv[]) {
    // argc: 명령줄 인수의 개수
    // argv: 명령줄 인수 배열 (argv[0]는 실행 파일 이름)

    // 첫 번째 인수(argv[0])는 프로그램의 이름이므로 두 번째 인수부터 출력
    for (int i = 1; i < argc; i++) {
        std::cout << argv[i] << std::endl;
    }

    return 0;
}
