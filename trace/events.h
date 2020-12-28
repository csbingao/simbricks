#pragma once

class event {
  public:
    uint64_t ts;

    event(uint64_t ts_)
        : ts(ts_)
    {
    }

    virtual void dump(std::ostream &out) = 0;
};

class EHostCall : public event {
  public:
    const std::string &fun;

    EHostCall(uint64_t ts_, const std::string &fun_)
        : event(ts_), fun(fun_)
    {
    }

    virtual void dump(std::ostream &out)
    {
        out << ts << ": H.CALL " << fun << std::endl;
    }
};

class EHostMsiX : public event {
  public:
    uint16_t vec;

    EHostMsiX(uint64_t ts_, uint16_t vec_)
        : event(ts_), vec(vec_)
    {
    }

    virtual void dump(std::ostream &out)
    {
        out << ts << ": H.MSIX " << vec << std::endl;
    }
};

class EHostDmaR : public event {
  public:
    uint64_t id;
    uint64_t addr;
    uint64_t size;

    EHostDmaR(uint64_t ts_, uint64_t id_, uint64_t addr_, uint64_t size_)
        : event(ts_), id(id_), addr(addr_), size(size_)
    {
    }

    virtual void dump(std::ostream &out)
    {
        out << ts << ": H.DMAR id=" << id << " addr=" << addr << " size=" <<
            size << std::endl;
    }
};

class EHostDmaW : public event {
  public:
    uint64_t id;
    uint64_t addr;
    uint64_t size;

    EHostDmaW(uint64_t ts_, uint64_t id_, uint64_t addr_, uint64_t size_)
        : event(ts_), id(id_), addr(addr_), size(size_)
    {
    }

    virtual void dump(std::ostream &out)
    {
        out << ts << ": H.DMAW id=" << id << " addr=" << addr << " size=" <<
            size << std::endl;
    }
};

class EHostDmaC : public event {
  public:
    uint64_t id;

    EHostDmaC(uint64_t ts_, uint64_t id_)
        : event(ts_), id(id_)    {
    }

    virtual void dump(std::ostream &out)
    {
        out << ts << ": H.DMAC id=" << id << std::endl;
    }
};

class EHostMmioR : public event {
  public:
    uint64_t id;
    uint64_t addr;
    uint64_t size;

    EHostMmioR(uint64_t ts_, uint64_t id_, uint64_t addr_, uint64_t size_)
        : event(ts_), id(id_), addr(addr_), size(size_)
    {
    }

    virtual void dump(std::ostream &out)
    {
        out << ts << ": H.MMIOR id=" << id << " addr=" << addr << " size=" <<
            size << std::endl;
    }
};

class EHostMmioW : public event {
  public:
    uint64_t id;
    uint64_t addr;
    uint64_t size;

    EHostMmioW(uint64_t ts_, uint64_t id_, uint64_t addr_, uint64_t size_)
        : event(ts_), id(id_), addr(addr_), size(size_)
    {
    }

    virtual void dump(std::ostream &out)
    {
        out << ts << ": H.MMIOW id=" << id << " addr=" << addr << " size=" <<
            size << std::endl;
    }
};

class EHostMmioC : public event {
  public:
    uint64_t id;

    EHostMmioC(uint64_t ts_, uint64_t id_)
        : event(ts_), id(id_)    {
    }

    virtual void dump(std::ostream &out)
    {
        out << ts << ": H.MMIOC id=" << id << std::endl;
    }
};