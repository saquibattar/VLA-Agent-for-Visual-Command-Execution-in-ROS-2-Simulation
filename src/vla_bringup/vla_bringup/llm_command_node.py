import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from groq import Groq
import os

class LLMCommandNode(Node):
    def __init__(self):
        super().__init__('llm_command_node')
        
        # Publisher for robot actions
        self.publisher = self.create_publisher(String, '/robot_action', 10)
        
        # Groq API setup
        self.groq_api_key = ""GROQ_API_KEY""
        
        # Check if API key is still the placeholder
        if self.groq_api_key == "PASTE_YOUR_GROQ_API_KEY_HERE" or len(self.groq_api_key) < 20:
            self.get_logger().error("Please set your Groq API key in the code!")
            self.get_logger().info("Get free API key from: https://console.groq.com")
            self.use_llm = False
        else:
            try:
                self.groq_client = Groq(api_key=self.groq_api_key)
                self.use_llm = True
                self.get_logger().info("LLM-VLA System Started")
                self.get_logger().info("Using Groq API with Llama-3.3 (70B parameters)")
                self.get_logger().info("Fast inference: ~0.5 seconds per command")
            except Exception as e:
                self.get_logger().error(f"Groq initialization failed: {e}")
                self.use_llm = False
        
        if not self.use_llm:
            self.get_logger().info("Fallback: Rule-based parser active")
    
    # ---------------------------------------------------
    def run_console(self):
        """Main console loop for accepting user commands"""
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("VLA COMMAND INTERFACE (LLM-Powered)")
        self.get_logger().info("="*60)
        self.get_logger().info("Examples:")
        self.get_logger().info("  - 'go to the red box'")
        self.get_logger().info("  - 'find the purple object'")
        self.get_logger().info("  - 'navigate to purple cube'")
        self.get_logger().info("  - 'pass through curtain to find blue object'")
        self.get_logger().info("  - 'stop' or 'halt'")
        self.get_logger().info("="*60 + "\n")
        
        while rclpy.ok():
            try:
                user_input = input("Enter Command: ").strip()
                
                # Skip empty inputs
                if user_input == "":
                    continue
                
                # Parse command using LLM or fallback
                if self.use_llm:
                    parsed_action = self.parse_with_groq_llm(user_input)
                else:
                    parsed_action = self.parse_with_rules(user_input)
                
                # Publish the parsed action
                msg = String()
                msg.data = parsed_action
                self.publisher.publish(msg)
                
                # Log the result
                if parsed_action == "UNKNOWN":
                    self.get_logger().warn(f"Could not parse: '{user_input}'")
                else:
                    self.get_logger().info(f"Command understood: {user_input}")
                    self.get_logger().info(f"Published action: {parsed_action}")
            
            except KeyboardInterrupt:
                self.get_logger().info("\nShutting down command interface...")
                break
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
    
    # ---------------------------------------------------
    def parse_with_groq_llm(self, text):
        """
        Use Groq API with Llama-3.3 70B for natural language understanding
        Extracts action-aware attributes as per VLA architecture
        """
        
        # Construct prompt for LLM
        system_prompt = """You are a robot command parser for a Vision-Language-Action (VLA) system.

Your task: Extract action-aware attributes from natural language commands and map them to robot actions.

IMPORTANT: Detect commands that involve PASSING THROUGH traversable obstacles (like curtains) to reach goals.

Respond with ONLY ONE of these exact words (nothing else):
- START_RED (if user wants to navigate/go/move/find red object/box/cube)
- START_PURPLE (if user wants to navigate/go/move/find purple/violet/magenta object/box/cube)
- START_BLUE_THROUGH_CURTAIN (if user wants to pass/go through curtain to reach blue object)
- STOP (if user wants to stop/halt/freeze/cancel/abort)
- UNKNOWN (if command is unclear)

Examples:
"go to red box" → START_RED
"find purple object" → START_PURPLE
"pass through the curtain to find the blue object" → START_BLUE_THROUGH_CURTAIN
"go through curtain to blue cube" → START_BLUE_THROUGH_CURTAIN
"stop now" → STOP

Respond with only the command word."""

        try:
            self.get_logger().info("Querying Groq LLM (Llama-3.3)...")
            
            # Call Groq API
            chat_completion = self.groq_client.chat.completions.create(
                messages=[
                    {
                        "role": "system",
                        "content": system_prompt
                    },
                    {
                        "role": "user",
                        "content": f"Command: {text}"
                    }
                ],
                model="llama-3.3-70b-versatile",
                temperature=0.1,
                max_tokens=30,
                top_p=0.9
            )
            
            # Extract response
            result = chat_completion.choices[0].message.content.strip().upper()
            
            self.get_logger().debug(f"LLM raw response: {result}")
            
            # Parse LLM output - BLUE added for paper's approach
            if 'START_RED' in result or 'START RED' in result:
                self.get_logger().info("LLM extracted: Navigate to RED object")
                return 'START_RED'
            elif 'START_PURPLE' in result or 'START PURPLE' in result:
                self.get_logger().info("LLM extracted: Navigate to PURPLE object")
                return 'START_PURPLE'
            elif 'START_BLUE_THROUGH_CURTAIN' in result or 'BLUE_THROUGH_CURTAIN' in result:
                self.get_logger().info("LLM extracted: Pass through CURTAIN to BLUE object")
                return 'START_BLUE_THROUGH_CURTAIN'
            elif 'STOP' in result:
                self.get_logger().info("LLM extracted: STOP command")
                return 'STOP'
            else:
                self.get_logger().warn(f"LLM response unclear: '{result}'")
                # Try fallback parser
                self.get_logger().info("Attempting fallback parser...")
                fallback_result = self.parse_with_rules(text)
                if fallback_result != 'UNKNOWN':
                    self.get_logger().info(f"Fallback succeeded: {fallback_result}")
                    return fallback_result
                return 'UNKNOWN'
        
        except Exception as e:
            self.get_logger().error(f"Groq API error: {str(e)[:100]}")
            self.get_logger().info("Using fallback parser...")
            return self.parse_with_rules(text)
    
    # ---------------------------------------------------
    def parse_with_rules(self, text):
        """
        Fallback rule-based parser with action-aware attribute extraction
        Ensures system reliability when LLM unavailable
        """
        
        text_lower = text.lower().strip()
        
        # Initialize attributes
        attributes = {
            'action': None,
            'target_color': None,
            'object_type': None,
            'traversable_obstacle': None
        }
        
        # ============= ACTION EXTRACTION =============
        action_keywords = {
            'navigate': ['go', 'move', 'navigate', 'drive', 'travel', 'head', 'proceed'],
            'find': ['find', 'search', 'locate', 'look for', 'seek'],
            'approach': ['approach', 'get close', 'reach', 'get to'],
            'pass_through': ['pass through', 'go through', 'traverse', 'cross'],
            'stop': ['stop', 'halt', 'freeze', 'cancel', 'abort', 'quit', 'end']
        }
        
        for action, keywords in action_keywords.items():
            if any(keyword in text_lower for keyword in keywords):
                attributes['action'] = action
                break
        
        # ============= CHECK FOR TRAVERSABLE OBSTACLES =============
        if 'curtain' in text_lower:
            attributes['traversable_obstacle'] = 'curtain'
        
        # ============= COLOR EXTRACTION =============
        color_keywords = {
            'RED': ['red', 'crimson', 'scarlet', 'ruby'],
            'PURPLE': ['purple', 'violet', 'magenta', 'lavender', 'plum'],
            'BLUE': ['blue', 'azure', 'navy']
        }
        
        for color, keywords in color_keywords.items():
            if any(keyword in text_lower for keyword in keywords):
                attributes['target_color'] = color
                break
        
        # ============= OBJECT TYPE EXTRACTION =============
        object_keywords = ['box', 'cube', 'object', 'block', 'thing', 'item', 'target']
        
        for obj_type in object_keywords:
            if obj_type in text_lower:
                attributes['object_type'] = obj_type
                break
        
        # Log extracted attributes
        if attributes['action'] or attributes['target_color']:
            self.get_logger().info(
                f"📝 Fallback extracted → Action: {attributes['action']}, "
                f"Color: {attributes['target_color']}, Obstacle: {attributes['traversable_obstacle']}"
            )
        
        # ============= COMMAND GENERATION =============
        
        # STOP has highest priority
        if attributes['action'] == 'stop':
            return "STOP"
        
        # CRITICAL: Detect pass-through-curtain-to-blue pattern (paper's approach)
        if (attributes['action'] in ['pass_through', 'navigate', 'find'] and 
            attributes['target_color'] == 'BLUE' and 
            attributes['traversable_obstacle'] == 'curtain'):
            return "START_BLUE_THROUGH_CURTAIN"
        
        # Navigation with action + color (original approach for RED/PURPLE)
        if attributes['action'] in ['navigate', 'find', 'approach'] and attributes['target_color']:
            return f"START_{attributes['target_color']}"
        
        # Implicit navigation - just color (original approach for RED/PURPLE)
        if attributes['target_color'] and not attributes['action']:
            return f"START_{attributes['target_color']}"
        
        return "UNKNOWN"


# ========================================================
def main(args=None):
    """Main entry point"""
    
    rclpy.init(args=args)
    node = LLMCommandNode()
    
    try:
        node.run_console()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
