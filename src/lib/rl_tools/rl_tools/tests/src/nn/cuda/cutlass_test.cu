
#include <cute/layout.hpp>


int main(){
    using namespace cute;
    Layout l2x2cm = make_layout(make_shape(_2{}, _2{}), make_stride(_1{}, _2{}));
    Layout l3x4rm = make_layout(make_shape(_3{}, _4{}), make_stride(_4{}, _1{}));
    Layout layout = blocked_product(l2x2cm, l3x4rm);
    print_layout(layout);
    for (int i = 0; i < size(layout); ++i) {
        print(layout(i));
        print(", ");
    }
    print("\n");
    print(layout(1, 1));
    print("\n");
}