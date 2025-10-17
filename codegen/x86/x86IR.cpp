#include "x86IR.h"

int main() {
    auto base = new Block{"base"};
    auto tru = new Block{"tru"};
    auto fals = new Block{"fals"};

    base->addTarget(tru);
    base->addTarget(fals);

    base->push<FAKE_DEF>(new Physical{RDI});
    base->push<MOV>(new Virtual(0), new Physical{RDI});
    base->push<MOVIMM>(new Virtual(1), 0);
    base->push<CMP>(new Virtual(0), new Virtual{1});
    base->push<JZ>(fals);
    base->push<JMP>(tru);

    fals->push<MOVIMM>(new Virtual(4), 1);
    fals->push<MOV>(new Virtual(5), new Virtual(0));
    fals->push<SUB>(new Virtual(5), new Virtual{4});

    fals->push<MOV>(new Physical{RDI}, new Virtual{5});

    fals->push<CALL>(std::initializer_list<BaseRegister*>{new Physical{RAX}}, std::initializer_list<BaseRegister*>{new Physical{RDI}}, std::initializer_list<BaseRegister*>{new Physical{RSI}, new Physical{RDX}, new Physical{RCX}}); // produces RAX, consumers RDI, clobbers {RDI, RSi, RDX, ...}
    fals->push<MOV>(new Virtual{7}, new Physical{RAX});
    fals->push<MOV>(new Virtual{8}, new Virtual{7});
    fals->push<MUL>(new Virtual{8}, new Virtual{0});
    fals->push<MOV>(new Physical{RAX}, new Virtual{8});
    fals->push<RET>(std::initializer_list<BaseRegister*>{new Physical{RAX}});


    tru->push<MOVIMM>(new Virtual(3), 1);
    tru->push<MOV>(new Physical(RAX), new Virtual{3});
    tru->push<RET>(std::initializer_list<BaseRegister*>{new Physical(RAX)});

    Graph graph;
    graph.blocks.push_back(base);
    graph.blocks.push_back(tru);
    graph.blocks.push_back(fals);

    graph.calcLiveRanges();

    std::set<size_t> virtIds = graph.getVirtIds();
    std::set<size_t> phyIds = graph.getPhyIds();

    std::map<size_t, std::set<size_t>> virtToVirt;
    std::map<size_t, std::set<size_t>> virtToPhy;

    for (auto self : virtIds) {
        for (auto other : virtIds) {
            auto ints = graph.virtVirtInt(self, other);
            if (ints) {
                virtToVirt[self].insert(other);
                virtToVirt[other].insert(self);
            }
        }
    }

    for (auto self : virtIds) {
        for (auto other : phyIds) {
            auto ints = graph.virtPhyInt(self, other);
            if (ints) {
                virtToPhy[self].insert(other);
            }
        }
    }

    /*    for (auto [a, stuff] : virtToVirt) {
        for (auto b : stuff) {
            if (a == b) continue;
            println("x{} -- x{}", a, b);
        }
    }*/

    size_t physVirtBase = 50'000;
    std::map<size_t, std::set<size_t>> virtToVirt2 = virtToVirt;
    for (auto [virt, physs] : virtToPhy) {
        for (auto phys : physs) {
            virtToVirt2[virt].insert(physVirtBase+phys);
            virtToVirt2[physVirtBase+phys].insert(virt);
        }
    }

    for (auto [a, stuff] : virtToVirt2) {
        for (auto b : stuff) {
            if (a == b) continue;
            println("x{} -- x{}", a, b);
        }
    }



    GraphColoring gc = GraphColoring::create(virtToVirt2);
    for (auto [virt, physs] : virtToPhy) {
        for (auto phys : physs) {
            gc.colorReg(physVirtBase+phys, phys);
        }
    }

    gc.regAlloc(0);

    for (auto [virt, phys] : gc.allocated) {
        println("x{}[xlabel={}]", virt, toName(phys));
        // std::cout << "solved " << virt << " <== " <<  toName(phys) << std::endl;
    }

    std::cout << "WHAT???" << std::endl;
}
