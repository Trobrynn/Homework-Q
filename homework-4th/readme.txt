基于科大讯飞实现中文和英文对话

视频杂音有些大请见谅


对话设计：

english:

发音人：catherine


  if((dataString.find("Hi") !=-1)||(dataString.find("Hello") !=-1))
  {
      char nameString[60] = "Hi. Nice to meet you.";
      text = nameString;
      std::cout<<text<<std::endl;
  }
  else if((dataString.find("introduce yourself")!=-1))
  {
      char helpString[50] = "Yeah, My name is Jack. And I'm a chat robot.";
      text = helpString;
      std::cout<<text<<std::endl;
  }
  else if((dataString.find("your hobby") != -1))
  {
      char helpString[40] = "I like talking to people.";      
      text = helpString;
      std::cout<<text<<std::endl;
  }
  else if((dataString.compare("Can you smile") == 0))
  {
      char helpString[80] = "Ha-ha.";      
      text = helpString;
      std::cout<<text<<std::endl;
  }
  else
  {
      text = msg->data.c_str();
  }


中文：

发音人：小燕


  if((dataString.find("介绍一下自己") !=-1))
  {
      char nameString[60] = "你好，我是聊天机器人，名叫小小。";
      text = nameString;
      std::cout<<text<<std::endl;
  }
  else if((dataString.find("月出于东山之上")!=-1))
  {
      char helpString[40] = "徘徊于斗牛之间。";
      text = helpString;
      std::cout<<text<<std::endl;
  }
  else if((dataString.find("讲个笑话") != -1))
  {
      char helpString[80] = "小王剪了中分，变成了小全。";      
      text = helpString;
      std::cout<<text<<std::endl;
  }
  else
  {
      text = msg->data.c_str();
  }
