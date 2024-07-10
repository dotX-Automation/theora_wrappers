# theora_wrappers

Wrappers for Theora publisher and subscriber.

## Contents

Two classes:

- `TheoraWrappers::Publisher`
- `TheoraWrappers::Subscriber`

They handle the two ends of a Theora stream, and behave like an `image_transport::Publisher` and `image_transport::Subscriber` respectively.

The main difference is that they wrap such objects, embedding a reset service that is used to reset the stream (*i.e.*, the publishing end) when a new subscriber connects. This is necessary because the `theora` plugin in `image_transport` does not support header retransmissions, and thus the stream must be reset when a new subscriber connects to an ongoing stream.

## Limitations

First and foremost, if while reading this you feel like this is a huge hack, it's because it is. The `image_transport` plugin system is not designed to support this kind of behavior, and thus we had to resort to some ugly hacks to make it work.

The best solution here would have been to reimplement the `theora` transport plugin to handle header retransmissions, but that would have been a lot of work, which the authors felt was not worth the effort given the age of the Theora codec. Such time would have been better spent on implementing a new plugin that supports some more modern, hardware-accelerated codec. We needed a quick solution, felt like Theora might work, did the absolute minimum to make it work, and moved on.

Anyway, here are the limitations:

- **The publisher appears to work well**: it has been tested even over WiFi with HD streams and it works fine. It always appears to reset the stream when a new subscriber connects.
- **The subscriber might not always work**: sometimes when it is created it receives a *bad header packet* and cannot show the stream until it is reset. Since the underlying `theora` subscriber gives no notification of this, the wrapper cannot do anything about it. This is a known issue with the `theora` plugin, and it is not clear what causes it. It appears to happen more often when the stream is reset, but it is not clear if this is the cause or just a coincidence. It is also not clear if this is a problem with the plugin itself, or with the Theora codec. In any case, it is not a problem with the wrapper, and it is not clear how to fix it. Since these wrappers are mainly intended for visualization purposes, there is no need to fix this issue, as the stream can be reset manually by the user.

## Requirements

Builds on ROS 2 Humble Hawksbill.

Requires `image_transport` and `image_transport_plugins`.

See [`package.xml`](package.xml) for more information.

---

## Copyright and License

Copyright 2024 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.
