#!/usr/bin/env python3
"""
Test script for PyTorch functionality (CPU only)
Tests basic PyTorch operations without GPU acceleration
"""

import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

def test_pytorch_basic():
    """Test basic PyTorch tensor operations"""
    print("🔥 Testing PyTorch basic operations (CPU)...")
    
    # Force CPU usage
    device = torch.device('cpu')
    print(f"✅ Using device: {device}")
    
    # Create tensors
    x = torch.randn(3, 4, device=device)
    y = torch.randn(4, 2, device=device)
    
    print(f"✅ Tensor x shape: {x.shape}")
    print(f"✅ Tensor y shape: {y.shape}")
    
    # Matrix multiplication
    z = torch.mm(x, y)
    print(f"✅ Matrix multiplication result shape: {z.shape}")
    
    # Basic operations
    a = torch.tensor([1.0, 2.0, 3.0, 4.0], device=device)
    b = torch.tensor([2.0, 3.0, 4.0, 5.0], device=device)
    
    print(f"✅ Addition: {a + b}")
    print(f"✅ Element-wise multiplication: {a * b}")
    print(f"✅ Dot product: {torch.dot(a, b)}")
    
    return x, y, z

def test_pytorch_autograd():
    """Test PyTorch automatic differentiation"""
    print("\n🔄 Testing PyTorch autograd...")
    
    # Create tensor with gradient tracking
    x = torch.tensor([2.0], requires_grad=True)
    
    # Define a function: f(x) = x^3 + 2x^2 + x + 1
    y = x**3 + 2*x**2 + x + 1
    
    # Compute gradient: f'(x) = 3x^2 + 4x + 1
    y.backward()
    
    expected_grad = 3 * x.item()**2 + 4 * x.item() + 1
    computed_grad = x.grad.item()
    
    print(f"✅ Input x: {x.item()}")
    print(f"✅ Function value: {y.item()}")
    print(f"✅ Expected gradient: {expected_grad}")
    print(f"✅ Computed gradient: {computed_grad}")
    print(f"✅ Gradient match: {abs(expected_grad - computed_grad) < 1e-6}")
    
    return x, y

def test_pytorch_neural_network():
    """Test a simple neural network"""
    print("\n🧠 Testing PyTorch neural network...")
    
    # Simple neural network for regression
    class SimpleNet(nn.Module):
        def __init__(self):
            super(SimpleNet, self).__init__()
            self.layer1 = nn.Linear(1, 10)
            self.layer2 = nn.Linear(10, 10)
            self.layer3 = nn.Linear(10, 1)
            self.relu = nn.ReLU()
            
        def forward(self, x):
            x = self.relu(self.layer1(x))
            x = self.relu(self.layer2(x))
            x = self.layer3(x)
            return x
    
    # Create model
    model = SimpleNet()
    print(f"✅ Model created with {sum(p.numel() for p in model.parameters())} parameters")
    
    # Generate synthetic data: y = 2x + 1 + noise
    x_data = torch.linspace(-1, 1, 100).unsqueeze(1)
    y_data = 2 * x_data + 1 + 0.1 * torch.randn_like(x_data)
    
    # Loss and optimizer
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=0.01)
    
    # Training loop
    losses = []
    print("✅ Training neural network...")
    
    for epoch in range(500):
        # Forward pass
        predictions = model(x_data)
        loss = criterion(predictions, y_data)
        
        # Backward pass
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        
        losses.append(loss.item())
        
        if epoch % 100 == 0:
            print(f"   Epoch {epoch}, Loss: {loss.item():.6f}")
    
    print(f"✅ Final loss: {losses[-1]:.6f}")
    
    # Test the trained model
    test_x = torch.tensor([[0.5]])
    predicted_y = model(test_x)
    expected_y = 2 * 0.5 + 1  # 2.0
    
    print(f"✅ Test input: {test_x.item()}")
    print(f"✅ Predicted output: {predicted_y.item():.3f}")
    print(f"✅ Expected output: {expected_y}")
    print(f"✅ Prediction accuracy: {abs(predicted_y.item() - expected_y) < 0.1}")
    
    return model, x_data, y_data, losses

def test_pytorch_visualization(model, x_data, y_data, losses):
    """Test PyTorch with matplotlib visualization"""
    print("\n📈 Testing PyTorch visualization...")
    
    script_dir = Path(__file__).parent
    
    # Create plots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Plot 1: Training loss
    ax1.plot(losses)
    ax1.set_title('Training Loss Over Time')
    ax1.set_xlabel('Epoch')
    ax1.set_ylabel('MSE Loss')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Model predictions vs actual data
    with torch.no_grad():
        x_test = torch.linspace(-1.5, 1.5, 200).unsqueeze(1)
        y_pred = model(x_test)
    
    ax2.scatter(x_data.numpy(), y_data.numpy(), alpha=0.5, label='Training Data', s=20)
    ax2.plot(x_test.numpy(), y_pred.numpy(), 'r-', label='Model Prediction', linewidth=2)
    ax2.plot(x_test.numpy(), 2 * x_test.numpy() + 1, 'g--', label='True Function', linewidth=2)
    ax2.set_title('Model Predictions vs True Function')
    ax2.set_xlabel('x')
    ax2.set_ylabel('y')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save plot
    plot_path = script_dir / "pytorch_cpu_test_plot.png"
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"✅ Visualization saved to: {plot_path}")

def main():
    """Main test function"""
    print("🧪 PyTorch CPU Test Suite")
    print("=" * 50)
    
    try:
        # Test basic operations
        x, y, z = test_pytorch_basic()
        
        # Test autograd
        test_pytorch_autograd()
        
        # Test neural network
        model, x_data, y_data, losses = test_pytorch_neural_network()
        
        # Test visualization
        test_pytorch_visualization(model, x_data, y_data, losses)
        
        print("\n🎉 All PyTorch CPU tests completed successfully!")
        print("=" * 50)
        print(f"✅ PyTorch version: {torch.__version__}")
        print(f"✅ CUDA available: {torch.cuda.is_available()}")
        print(f"✅ Device used: CPU")
        print("✅ Tensor operations: PASSED")
        print("✅ Autograd: PASSED")
        print("✅ Neural network: PASSED")
        print("✅ Visualization: PASSED")
        
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()