#include <msgpack.hpp>
#include <vector>
#include <string>
#include <iostream>

int main(void) {
        // serializes this object.
        std::vector<float> vec{{0.01,0.04}};
        vec.push_back(0.01);
        vec.push_back(0.004);

        // serialize it into simple buffer.
        msgpack::sbuffer sbuf;
        msgpack::pack(sbuf, vec);

        // deserialize it.
        msgpack::object_handle oh =
            msgpack::unpack(sbuf.data(), sbuf.size());

        // print the deserialized object.
        msgpack::object obj = oh.get();
        std::cout << obj << std::endl;  //=> ["Hello", "MessagePack"]

        // convert it into statically typed object.
        std::vector<float> rvec;
        obj.convert(rvec);
}
