//===- Hello.cpp - Example code from "Writing an LLVM Pass" ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements two versions of the LLVM "Hello World" pass described
// in docs/WritingAnLLVMPass.html
//
//===----------------------------------------------------------------------===//

#include <cxxabi.h>
#include "llvm/ADT/Statistic.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/DebugInfoMetadata.h"
#include "llvm/Pass.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#define DEBUG_TYPE "uxas"

//STATISTIC(UxasCounter, "Counts number of functions greeted");

namespace {
  // Uxas - The first implementation, without getAnalysisUsage.
  struct Uxas : public ModulePass {
    static char ID; // Pass identification, replacement for typeid
    Uxas() : ModulePass(ID) {}

    //-- return the class name given a method of the class. if F does
    //-- not belong to a class return an empty string as name.
    StringRef getClassName(Function &F)
    {
      if(F.getFunctionType()->getNumParams()) {
        if(auto pt = dyn_cast<PointerType>(F.getFunctionType()->getParamType(0))) {
          if(auto *st = dyn_cast<StructType>(pt->getElementType()))
            //-- the name always starts with "class.", so get rid of that
            return st->getName().substr(6);
        }
      }

      return "";
    }

    //-- return demangled name
    std::string demangle(const StringRef &mangled)
    {
      int status = 0;
      std::string res(abi::__cxa_demangle(mangled.data(), 0, 0, &status));
      return res.substr(0, res.find('['));
    }

    //-- print all messages that function F subscribes to
    void printSubs(Function &F)
    {
      StringRef className = getClassName(F);
      if(className == "") return;

      errs() << "inputs\n";

      //-- iterate over all basic blocks
      for(const BasicBlock &bb : F.getBasicBlockList()) {
        //-- iterate over statements
        for(const Instruction &ins : bb.getInstList()) {
        	//ins.dump();
          if(auto *II = dyn_cast<const CallInst>(&ins)) {
            const Function *cf = II->getCalledFunction();
            if(cf == NULL || !cf->getName().contains("addSubscriptionAddress")) continue;

            //-- the first argument to a method call is always the
            //-- this pointer. so get the second argument.
            auto *arg = II->getArgOperand(1);

            if(auto *Const = dyn_cast<Constant>(arg))
              errs() << demangle(Const->getName()) << '\n';
            else if(auto *II = dyn_cast<InvokeInst>(arg)) {
              errs() << demangle(II->getCalledFunction()->getName()) << "()\n";
            }
          }else if(auto *II = dyn_cast<const InvokeInst>(&ins)) {
              const Function *cf = II->getCalledFunction();
              if(cf == NULL || !cf->getName().contains("addSubscriptionAddress")) continue;

              //-- the first argument to a method call is always the
              //-- this pointer. so get the second argument.
              auto *arg = II->getArgOperand(1);

              if(auto *Const = dyn_cast<Constant>(arg))
                errs() << demangle(Const->getName()) << '\n';
              else if(auto *II = dyn_cast<InvokeInst>(arg)) {
                errs() << demangle(II->getCalledFunction()->getName()) << "()\n";
              }
            }
        }
      }
    }

    //-- print all messages that function F publishes
    void printPubs(Function &F)
    {
//      StringRef className = getClassName(F);
//      if(className != serviceName) return;

      //-- iterate over all basic blocks
      for(const BasicBlock &bb : F.getBasicBlockList()) {
        //-- iterate over statements
        for(const Instruction &ins : bb.getInstList()) {
          if(auto *II = dyn_cast<const InvokeInst>(&ins)) {
            const Function *cf = II->getCalledFunction();
            if(cf == NULL) continue;

            //-- get the argument that corresponds to the message
            //-- being sent
            Value *arg = NULL;
            if(cf->getName().contains("sendSharedLmcpObjectBroadcastMessage"))
              arg = II->getArgOperand(1);
            else if(cf->getName().contains("sendSharedLmcpObjectLimitedCastMessage"))
              arg = II->getArgOperand(2);
            if(!arg) continue;

            //-- the first argument to a method call is always the
            //-- this pointer. so get the second argument.
            for(auto u : arg->users()) {
              if(auto *CI = dyn_cast<CallInst>(u)) {
                if(Function *CF = CI->getCalledFunction()) {
                  if(CF->getName().contains("static_pointer_cast")) {
                    std::string dn = demangle(CF->getName());
                    size_t pos1 = dn.find(',');
                    if(pos1 == std::string::npos) continue;
                    size_t pos2 = dn.find('>', pos1);
                    if(pos2 == std::string::npos) continue;
                    errs() << dn.substr(pos1+2, pos2-pos1-2) << '\n';
                  }
                  else if(CF->getName().contains("shared_ptr")) {
                    std::string dn = demangle(CF->getName());
                    size_t pos1 = dn.find("shared_ptr<");
                    if(pos1 == std::string::npos) continue;

                    pos1 = dn.find("shared_ptr<", pos1+1);
                    if(pos1 == std::string::npos) continue;

                    pos1 = dn.find("shared_ptr<", pos1+1);
                    if(pos1 == std::string::npos) continue;

                    size_t pos2 = dn.find('>', pos1);
                    if(pos2 == std::string::npos) continue;
                    errs() << dn.substr(pos1+11, pos2-pos1-11) << '\n';
                  }
                }
              }else if(auto *CI = dyn_cast<InvokeInst>(u)) {
                  if(Function *CF = CI->getCalledFunction()) {
                    if(CF->getName().contains("static_pointer_cast")) {
                      std::string dn = demangle(CF->getName());
                      size_t pos1 = dn.find(',');
                      if(pos1 == std::string::npos) continue;
                      size_t pos2 = dn.find('>', pos1);
                      if(pos2 == std::string::npos) continue;
                      errs() << dn.substr(pos1+2, pos2-pos1-2) << '\n';
                    }
                    else if(CF->getName().contains("shared_ptr")) {
                      std::string dn = demangle(CF->getName());
                      size_t pos1 = dn.find("shared_ptr<");
                      if(pos1 == std::string::npos) continue;

                      pos1 = dn.find("shared_ptr<", pos1+1);
                      if(pos1 == std::string::npos) continue;

                      pos1 = dn.find("shared_ptr<", pos1+1);
                      if(pos1 == std::string::npos) continue;

                      size_t pos2 = dn.find('>', pos1);
                      if(pos2 == std::string::npos) continue;
                      errs() << dn.substr(pos1+11, pos2-pos1-11) << '\n';
                    }
                  }
                }
            }
          }
        }
      }
    }

    //-- name of the service class being analyzed
    StringRef serviceName;

    //-- the top level method

    //method to get the source line number of an instruction
    unsigned getLineNumber(const Instruction* I){
    	SmallVector<std::pair<unsigned, MDNode *>, 8> vec;
    	I->getAllMetadata(vec);
    	for (const auto & node : vec){
    		llvm::MDNode* const second = node.second;
    		if(isa<DILocation>(second)){
    			const DILocation * loc = dyn_cast<DILocation>(second);
    			return loc->getLine();
    		}
    	}
    	return 0;
    }


    bool runOnModule(Module &M) override
    {
      //-- first get the name of the service class by looking for the
      //-- configure method and seeing which class it belongs to
      for(Function &F : M.getFunctionList()) {
    	  if(!F.getName().contains("configure")) continue;
        StringRef className = getClassName(F);
        if(className == "") continue;
        serviceName = className;
        printSubs(F);
        break;
      }

      errs() << "outputs\n";
      //-- now extract the pub-sub information
      for(Function &F : M.getFunctionList()) {
        if(!F.getName().contains("configure")) {
        	printPubs(F);
        }
      }

      return false;
    }
  };
}

char Uxas::ID = 0;
static RegisterPass<Uxas> X("uxas", "Uxas Pass");
