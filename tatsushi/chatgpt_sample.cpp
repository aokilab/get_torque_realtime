#include <iostream>
#include <vector>

int main() {
    // 空のベクターの作成
    std::vector<int> numbers;

    // 要素の追加
    numbers.push_back(10);
    numbers.push_back(20);
    numbers.push_back(30);

    // ベクターのサイズ
    std::cout << "Vector size: " << numbers.size() << std::endl;

    // ベクターの要素へのアクセス
    std::cout << "First element: " << numbers[0] << std::endl;

    // ベクターの走査
    std::cout << "Elements: ";
    for (const auto& num : numbers) {
        std::cout << num << " ";
    }
    std::cout << std::endl;

    return 0;
}