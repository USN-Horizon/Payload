#include "mavlink/HorizonDialect/HorizonDialect.hpp"

namespace mavlink {

const mavlink_msg_entry_t *mavlink_get_msg_entry(uint32_t msgid)
{
    static const auto &entries = mavlink::HorizonDialect::MESSAGE_ENTRIES;

    uint32_t low = 0, high = static_cast<uint32_t>(entries.size()) - 1;
    while (low < high) {
        uint32_t mid = (low + 1 + high) / 2;
        if (msgid < entries[mid].msgid) {
            high = mid - 1;
            continue;
        }
        if (msgid > entries[mid].msgid) {
            low = mid;
            continue;
        }
        low = mid;
        break;
    }
    if (entries[low].msgid != msgid) {
        return nullptr;
    }
    return &entries[low];
}

} // namespace mavlink
